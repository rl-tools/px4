Using px4: 7f251d093568179209fa724d155ef0301c3657cf

optional: Disable CBRK_BUZZER (INT32)
Add CONFIG_MODULES_BACKPROP_TOOLS_BENCHMARK=y in the boards px4config.default
Main cmakelist:
  Move the loading of external modules to after then "include(px4_add_library)"
  Change to c++17 in the main

Build with: make mro_pixracerpro_default EXTERNAL_MODULES_LOCATION=/home/jonas/phd/projects/rl_for_control/px4/external_modules
Flash with: make mro_pixracerpro_default upload


Usage: Go into the QGroundControl console
Start module: backprop_tools_benchmark start
Watch printed output: dmesg