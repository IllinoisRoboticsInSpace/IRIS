# Development Environment

## Current Issues

- librealsense2 is not available on Macs
- Visualization isn't working on any platform
    - Developing w/ Gazebo or RViz is not really possible

## Docker Hub Repository

- iris_uiuc is the name of the repository
- iris is the name of the main image.

## Instructions to Push a New Version/Add a New Architecture

```bash
docker tag iris rushils4/iris_uiuc:latest
docker image push rushils4/iris_uiuc:latest
```

## Contributors
- Author: Daniel Philipov | <dp33@illinois.edu>
- Active Maintainers:
    - Rama Rao Vencharla | <ramarao3@illinois.edu>
    - Rushil Shah | <rushils4@illinois.edu>