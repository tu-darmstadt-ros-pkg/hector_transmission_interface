// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <transmission_interface/transmission_loader.hpp>

// The whole point of exporting a plugin description is that ros2_control can load the loader by
// name. The description names the library the class lives in, and nothing checks that name until
// something tries: a wrong one fails at load time with "class not found", in whatever consumer
// happens to go through pluginlib rather than constructing the loader directly.
TEST( PluginExport, TheExportedTransmissionLoaderCanBeLoadedByName )
{
  pluginlib::ClassLoader<transmission_interface::TransmissionLoader> loader(
      "transmission_interface", "transmission_interface::TransmissionLoader" );

  const auto declared = loader.getDeclaredClasses();
  ASSERT_NE( std::find( declared.begin(), declared.end(),
                        "hector_transmission_interface/AdjustableOffsetTransmission" ),
             declared.end() )
      << "The transmission is not declared to pluginlib at all.";

  std::shared_ptr<transmission_interface::TransmissionLoader> instance;
  ASSERT_NO_THROW( instance = loader.createSharedInstance(
                       "hector_transmission_interface/AdjustableOffsetTransmission" ) )
      << "The plugin description names a library the class is not in.";
  EXPECT_NE( instance, nullptr );
}
