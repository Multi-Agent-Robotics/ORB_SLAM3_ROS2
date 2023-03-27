set shell := ["bash", "-uc"]
# set shell := ["fish", "-c"]
# load .env file
set dotenv-load

cwd := `basename $(pwd)`

default:
    @just --list

alias b := build

build:
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select {{cwd}}


