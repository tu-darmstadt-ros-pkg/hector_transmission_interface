#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hector_transmission_interface/adjustable_offset_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtest/service_mock.hpp"

using namespace hector_transmission_interface;
using namespace hector_transmission_interface_msgs::srv;
using testing::DoubleEq;
using testing::ElementsAre;
using testing::Field;
using testing::Return;

class MockAdjustableOffset : public IAdjustableOffset
{
public:
  MOCK_METHOD( void, adjustOffset, ( double offset ), ( override ) );
  MOCK_METHOD( double, getOffset, (), ( const, override ) );
};

class AdjustableOffsetManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "test_node" );
    manager_ = std::make_shared<AdjustableOffsetManager>( node_ );

    // 1. Try to find the service. If it fails, stop the test immediately to avoid Segfault.
    auto service =
        rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
    ASSERT_NE( service, nullptr ) << "Service not found in rtest registry!";
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AdjustableOffsetManager> manager_;
};

TEST_F( AdjustableOffsetManagerTest, RobustOffsetCalculation )
{
  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  std::string joint_name = "flipper_joint";

  // Logic: NewOffset = External(1.2) - Internal(1.0) + CurrentOffset(0.5) = 0.7
  double current_internal_val = 1.0;
  double old_offset = 0.5;
  double external_meas = 1.2;
  double expected_new_offset = 0.7;

  // Setup Mock expectations for the internal transmission handles
  EXPECT_CALL( *mock_state, getOffset() ).WillRepeatedly( Return( old_offset ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );

  // Inject mocks into manager
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = [&]() { return current_internal_val; };
  manager_->add_managed_joint( joint_name, mj );

  // Retrieve the service mock via rtest
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  ASSERT_TRUE( service );

  // 1. Set up Request Header (rtest requirement)
  auto request_header = std::make_shared<rmw_request_id_t>();
  request_header->sequence_number = 1L;

  // 2. Set up Request Data
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name.push_back( joint_name );
  request->external_joint_measurements.position.push_back( external_meas );

  // 3. Set up Expected Response for validation
  AdjustTransmissionOffsets::Response expected_response;
  expected_response.success = true;
  expected_response.adjusted_offsets.push_back( expected_new_offset );
  // Optional: match message content if desired

  // 4. Expect that the service will call send_response with the correct data
  // Using ElementsAre to check the floating point vector content
  EXPECT_CALL(
      *service,
      send_response( *request_header,
                     testing::AllOf( Field( &AdjustTransmissionOffsets::Response::success, true ),
                                     Field( &AdjustTransmissionOffsets::Response::adjusted_offsets,
                                            ElementsAre( DoubleEq( expected_new_offset ) ) ) ) ) );

  // 5. Simulate the service call
  service->handle_request( request_header, request );
}

TEST_F( AdjustableOffsetManagerTest, CalculationWithOriginalState )
{
  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  std::string joint_name = "flipper_joint";

  // Current hardware state is 1.0, but original measurement state was 0.8
  // NewOffset = External(1.5) - OriginalInternal(0.8) + CurrentOffset(0.0) = 0.7
  double current_internal_val = 1.0;
  double original_internal_val = 0.8;
  double external_meas = 1.5;
  double expected_new_offset = 0.7;

  EXPECT_CALL( *mock_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );

  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = [&]() { return current_internal_val; };
  manager_->add_managed_joint( joint_name, mj );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name.push_back( joint_name );
  request->external_joint_measurements.position.push_back( external_meas );

  // Provide the original state to bypass the position_getter
  request->original_joint_state.name.push_back( joint_name );
  request->original_joint_state.position.push_back( original_internal_val );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, true ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Partial Data and Fallback Logic
 * Scenario: Request contains 2 joints. Joint A has an 'original_joint_state',
 * Joint B does not. Joint B should fallback to the position_getter.
 */
TEST_F( AdjustableOffsetManagerTest, PartialOriginalStateFallback )
{
  auto mock_a = std::make_shared<MockAdjustableOffset>();
  auto mock_b = std::make_shared<MockAdjustableOffset>();

  // Joint A: External(1.5) - Original(1.0) + Offset(0.0) = 0.5
  // Joint B: External(2.5) - Current(2.0) + Offset(0.0) = 0.5

  AdjustableOffsetManager::ManagedJoint mj_a, mj_b;
  mj_a.state_handle = mj_a.command_handle = mock_a;
  mj_a.position_getter = []() { return 999.0; }; // Should be ignored for A

  mj_b.state_handle = mj_b.command_handle = mock_b;
  mj_b.position_getter = []() { return 2.0; }; // Used for B

  manager_->add_managed_joint( "joint_a", mj_a );
  manager_->add_managed_joint( "joint_b", mj_b );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "joint_a", "joint_b" };
  request->external_joint_measurements.position = { 1.5, 2.5 };

  // Only provide original state for joint_a
  request->original_joint_state.name = { "joint_a" };
  request->original_joint_state.position = { 1.0 };

  EXPECT_CALL( *mock_a, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_a, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 2 );
  EXPECT_CALL( *mock_b, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_b, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 2 );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, true ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Invalid Joint Names
 * Scenario: Request contains a joint name that the manager doesn't know about.
 * Manager should return success=false or at least not crash/adjust wrong joints.
 */
TEST_F( AdjustableOffsetManagerTest, NonExistentJointInRequest )
{
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "ghost_joint" };
  request->external_joint_measurements.position = { 1.0 };

  // We expect success = false because nothing was adjusted
  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, false ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Corrupted Data (NaN)
 * Scenario: External measurement is NaN. Manager should skip or handle gracefully.
 */
TEST_F( AdjustableOffsetManagerTest, HandlingNaNValues )
{
  auto mock = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mj.command_handle = mock;
  mj.position_getter = []() { return 1.0; };
  manager_->add_managed_joint( "nan_joint", mj );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "nan_joint" };
  request->external_joint_measurements.position = { std::numeric_limits<double>::quiet_NaN() };

  // The manager should NOT call adjustOffset if the input is NaN
  EXPECT_CALL( *mock, adjustOffset( testing::_ ) ).Times( 0 );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, false ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Mismatched Array Sizes
 * Scenario: User sends 2 names but only 1 position.
 * This tests the robustness of the loop indices.
 */
TEST_F( AdjustableOffsetManagerTest, MismatchedArraySizes )
{
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "j1", "j2" };
  request->external_joint_measurements.position = { 1.0 }; // Only one!

  // Manager should handle this without out_of_bounds exception
  // Usually by iterating based on the smaller of the two sizes or the name size with a check.
  EXPECT_CALL( *service, send_response( *request_header, testing::_ ) ).Times( 1 );

  service->handle_request( request_header, request );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
