# hector_transmission_interface

## Adjustable Offset Transmission Interface
Extends the simple transmission interface to store/load offsets and adjust them. Can be used to detect and correct
slipping of a transmission, for example a belt or a chain.
Make sure to unload all controllers before changing the offsets!!

## Controller Orchestrator
Defines several convenience functions for interacting with the controller manager.
#### Smart Switching
Allows to activate a list of controllers and if necessary deactivate all controllers that are currently claiming a required command interface.
#### Get Active Controllers of Hardware Interface
Allows to get a list of all active controllers claiming joints of a specific hardware interface. Allows to deactivate all controllers
of the hardware interface.

