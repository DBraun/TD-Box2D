# TD-Box2D

This prepares [Box2D](https://github.com/erincatto/box2d) for usage in TouchDesigner [Custom Operators](https://docs.derivative.ca/Custom_Operators). Users can make their own plugins by copying from the existing projects inside the [TD-Box2D folder](https://github.com/DBraun/TD-Box2D/tree/master/TD-Box2D).

Currently implemented:
* [TD-Box2D-MouseDemo](https://www.instagram.com/p/Be6lsn1AM3N/)

## Installation

### Windows

Install [CMake](https://cmake.org/download/) and make sure it's in your system path. Then in this repo, open a cmd window and do the following:

```bash
mkdir build
cd build
cmake ..
```

Open `build/TD-Box2D.sln` and build in Release from the top window (Debug is not tested yet). Build a second time (this will be fixed eventually). Then press `F5` and TouchDesigner should open. This repo's `Plugins` folder should contain newly compiled DLLs.

### OSX

Not fully tested yet, but the Windows instructions might work.

### Linux

TouchDesigner isn't on Linux ;)

## Roadmap

* Your suggestion here (open a Github issue)

## License

Please observe the licenses of TouchDesigner, SFML, and Box2D.
