# max35103evkit2
board support for the MAX35103EVKIT2

![MAX35103EVKIT2 contents](/doc/contents.jpg)

## Overview

This repository contains the board support files for the MAX35103EVKIT2 and is used by application examples found elsewhere under the maxim-ic-flow github organization.  This could serve as a starting point for a new MAX35103EVKIT2 application.

For an example of how to use this library, see [volumetric](https://github.com/maxim-ic-flow/volumetric).

## Repository

Please note that this project uses git submodules.  The proper way to clone this repository is as follows:

```
git clone --recursive https://github.com/maxim-ic-flow/volumetric.git
```
To switch between branches:

```
git checkout <branch>
git submodule update --recursive --remote
```

## Branches

<i>master</i> contains the chip support library (CSL) for the MAX32620
<p><i>mbed</i> is intended for use with the ARM mbed environment and so does not contain the CSL
