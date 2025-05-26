# Running GeoScenario server on Windows 10/11 WSL2

1. Install a Linux distribution (e.g., `Ubuntu`) within the `WSL2` environment. 

Follow the [guide](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) until section "Run Linux GUI apps", which is optional.

*NOTE*: It is important to have the latest `WSL2` version installed because it contains `WSLg` required for GUI apps.
Installing a GUI app, such as `gedit` is not necessary but could be used to verify a working `WSL2` environment.
Contrary to some older guides, there's no need to install a separate X server and modify the variable `DISPLAY`.
If no GUI appears, `wsl --update` and `wsl --shutdown` should fix the issue.

2. Open the linux terminal, install `pixi`, and follow the README.
The pixi environment contains everything necessary to run on WSL2.
