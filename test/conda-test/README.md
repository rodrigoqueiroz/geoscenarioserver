# geoscenarioserver conda package installation and execution test

# pixi 

The workspace definition in `pixi.yaml` installs the TrueType variant of TK, because that requirement cannot be specified in the geoscenario conda package itself as of March 2026.

# micromamba

For micromamba, we create a local channel from which the package can be installed in `channel/linux-64/`.
The channel has to be first indexed using `rattler-index fs conda-test/channel` to make the package available.