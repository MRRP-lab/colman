## Setup
*The following setup was tested on Ubuntu 25.10 with no errors*

1. Clone this repository.
1. Once in the repository, switch to the pixi branch with git checkout:
    ```
    git checkout pixi
    ```
1. Install Pixi:
    ```
    curl -fsSL https://pixi.sh/install.sh | sh
    ```
1. Build the Pixi project:
    ```
    pixi run build
    ```
1. Launch Gazebo and RViz:
    ```
    pixi run sim
    ``` 
