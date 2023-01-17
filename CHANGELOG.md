# Changelog
* **1.3.0** (Jan. 2023)
  * Added global printing and tracking tools.
  * [Breaking Compatibility] Refactored access to global settings.
* **1.2.3** (Dec. 2022)
  * Cleaned up -WAll compiler warnings.
* **1.2.2** (Jan. 2022)
  * Fixed double printing of text if default detection is activated.
* **1.2.1** (Nov. 2021)
  * Variable configs are now by default optional (left un-initialized if the params are not set).
  * Added CI for build tests on Ubuntu 18 and 20.
* **1.2.0** (Nov. 2021)
  * Added variable configs.
* **1.1.7** (Sept. 2021)
  * Can now optionally display units when printing configs.
* **1.1.6** (Sept 2021)
  * Can now indicate which values are defaults, which ones are different.
  * Refactored Settings to be global (rather than for each config) and dynamic (can be changed during program execution and not only when creating configs).
* **1.1.5** and below
  * Initial release.
  * Functionality for merged setup.
  * Getting configs from ROS.
  * Factories for ROS and non-ROS object creation.
