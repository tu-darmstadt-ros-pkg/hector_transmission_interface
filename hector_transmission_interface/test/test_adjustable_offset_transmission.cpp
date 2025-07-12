//
// Created by aljoscha-schmidt on 7/12/25.
//
#include <fstream>
#include <gtest/gtest.h>
#include <hector_transmission_interface/adjustable_offset_transmission.hpp>

using namespace hector_transmission_interface;

class AdjustableOffsetTransmissionTest : public ::testing::Test
{
protected:
  std::string joint_name = "test_joint_offset";
  std::string secondary_joint_name = "test_joint_offset_secondary";
  std::filesystem::path offset_file_1;
  std::filesystem::path offset_file_2;

  void SetUp() override
  {
    offset_file_1 = std::filesystem::path( std::getenv( "HOME" ) ) /
                    ".ros/dynamic_offset_transmissions" / ( joint_name + ".txt" );
    // Clean before each test
    std::filesystem::remove( offset_file_1 );
    offset_file_2 = std::filesystem::path( std::getenv( "HOME" ) ) /
                    ".ros/dynamic_offset_transmissions" / ( secondary_joint_name + ".txt" );
    // Clean before each test
    std::filesystem::remove( offset_file_2 );
  }

  void TearDown() override
  {
    std::filesystem::remove( offset_file_1 );
    std::filesystem::remove( offset_file_2 );
  }

  bool fileContainsValue( double expected )
  {
    std::ifstream file( offset_file_1 );
    double val = 0.0;
    file >> val;
    bool near = std::abs( val - expected ) < 1e-9;
    if ( !near )
      std::cout << "Expected: " << expected << ", got: " << val << std::endl;
    return near;
  }
};

TEST_F( AdjustableOffsetTransmissionTest, LoadAndSaveOffset )
{
  // 1) Create instance with initial offset
  AdjustableOffsetTransmission trans( joint_name, 1.0, 0.123 );
  EXPECT_NEAR( trans.get_joint_offset(), 0.123, 1e-9 );
  EXPECT_TRUE( fileContainsValue( 0.123 ) );

  // 2) Adjust offset and check file was updated
  trans.adjustTransmissionOffset( 0.456 );
  EXPECT_NEAR( trans.get_joint_offset(), 0.456, 1e-9 );
  EXPECT_TRUE( fileContainsValue( 0.456 ) );

  // 3) Reload new instance from file, check offset persists
  AdjustableOffsetTransmission trans2( joint_name, 1.0 );
  EXPECT_NEAR( trans2.get_joint_offset(), 0.456, 1e-9 );

  // 4) Load second instance with different joint name -> make sure different files are used
  AdjustableOffsetTransmission trans3( secondary_joint_name, 1.0, 2.0 );
  EXPECT_NEAR( trans3.get_joint_offset(), 2.0,
               1e-9 ); // Should be 2.0 -> no file exists for this joint use provided default
}
