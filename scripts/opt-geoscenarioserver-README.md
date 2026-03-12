# Creating a self-contained GSS environment for both standalone and ROS2

The following steps create `opt-geoscenarioserver-<version>.tar.zstd` ready to be used with the `opt-geoscenarioserver-install.bash` script.
The environment does not require `pixi` or `micromamba`; however, it must be installed to `/opt/geoscenarioserver` because conda environments are not relocatable.

1. Create `*.conda` packages. Execute the following command in the terminal to create the packages:

```bash
bash scripts/build_all_conda_packages.bash
```

2. Upload the `*.conda` packages to the `conda-packages/linux-64` channel on the server and index the channel. Execute on the server:

```bash
pixi global install rattler-index
rattler-index fs conda-packages/
```

3. Create the `opt-geoscenarioserver-<version>.tar.zstd` and upload to the server. 
Execute locally (`<version>` is optional, defaults to `0.1.0`):

```bash
bash scripts/opt-geoscenarioserver-create.bash [<version>]
```
Warning: this will also install the environment to `/opt/geoscenarioserver` locally.

4. On the server, symlink the `opt-geoscenarioserver-latest.tar.zstd` to the uploaded `opt-geoscenarioserver-<version>.tar.zstd`:

Now users can use `opt-geoscenarioserver-install.bash` to install the environment on their local machine. 
The script will download the `opt-geoscenarioserver-latest.tar.zstd` from the server and install it to `/opt/geoscenarioserver`.
