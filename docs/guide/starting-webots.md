## Starting Webots

The first time you start Webots, it will open the "Welcome to Webots!" menu with a list of possible starting points.

### Linux

Open a terminal and type `webots` to launch Webots.

### macOS

Open the directory in which you installed the Webots package and double-click on the Webots icon.

### Windows

On Windows 10 and Windows 7, open the `Start` menu, go to the `Program Files / Cyberbotics` menu and click on the `Webots {{ webots.version.full }}` menu item.

On Windows 8, open the `Start` screen, scroll to the screen's right until you spot the Cyberbotics section and click on the `Webots` icon.

You can also start Webots from a DOS console (`cmd.exe`) by typing `webots` or `webots.exe`.
This command works only if executed from the `C:\Program Files\Webots\msys64\mingw64\bin` directory (assuming Webots was installed in `C:\Program Files`).
It can also work from any directory if the above directory was added to your `Path` environment variable.
**Note:** Calling `webots-bin.exe` from a DOS console may not work, as some settings may be missing.
**Note:** Calling `webotsw.exe` from a DOS console launches Webots in the background and returns immediately.

### Command Line Arguments

Following command line options are available when starting Webots from a Terminal (Linux/Mac) or a Command Prompt (Windows):

```
Usage: webots [options] [worldfile]

Options:

  --help
    Display this help message and exit.

  --version
    Display version information and exit.

  --sysinfo
    Display information about the system and exit.

  --mode=<mode>
    Choose the startup mode, overriding application preferences. The <mode>
    argument must be either pause, realtime, run or fast.

  --fullscreen
    Start Webots in fullscreen.

  --minimize
    Minimize the Webots window on startup.

  --batch
    Prevent Webots from creating blocking pop-up windows.

  --stdout
    Redirect the stdout of the controllers to the terminal.

  --stderr
    Redirect the stderr of the controllers to the terminal.

  --stream[="key[=value];..."]
    Start the Webots streaming server. Parameters may be
    given as an option:
      port=1234          - Start the streaming server on port 1234.
      mode=<x3d|mjpeg>   - Specify the streaming mode: x3d (default) or mjpeg.
      monitorActivity    - Print a dot '.' on stdout every 5 seconds.
      disableTextStreams - Disable the streaming of stdout and stderr.

  --log-performance=<file>[,<steps>]
    Measure the performance of Webots and log it in the file specified in the
    <file> argument. The optional <steps> argument is an integer value that
    specifies how many steps are logged. If the --sysinfo option is used, the
    system information is prepended into the log file.

Please report any bug to https://cyberbotics.com/bug
```

The optional `worldfile` argument specifies the name of a .wbt file to open.
If it is not specified, Webots attempts to open the most recently opened file.

The `--minimize` option is used to minimize (iconize) Webots window on startup.
This also skips the splash screen and the eventual Welcome Dialog.
This option can be used to avoid cluttering the screen with windows when automatically launching Webots from scripts.
Note that Webots automatically uses the `Fast` mode when `--minimize` is specified.

The `--mode=<mode>` option can be used to start Webots in the specified simulation mode.
The four possible simulation modes are: `pause`, `realtime`, `run` and `fast`; they correspond to the simulation control buttons of Webots' graphical user interface.
This option overrides, but does not modify, the startup mode saved in Webots' preferences.
For example, type `webots --mode=pause filename.wbt` to start Webots in `pause` mode.

The `--sysinfo` option displays misc information about the current system on the standard output stream and quits Webots.

The `--stdout` and `--stderr` options have the effect of redirecting Webots console output to the calling terminal or process.
For example, this can be used to redirect the controllers output to a file or to pipe it to a shell command.
`--stdout` redirects the *stdout* stream of the controllers, while `--stderr` redirects the *stderr* stream.
Note that the *stderr* stream may also contain Webots error or warning messages.

The `--stream` option starts the Webots streaming server.
An option can be given to change the default parameters of the streaming server.
This option is a string containing a list of parameter keys and their values separated by semicolons.
The supported options are described in [this table](#streaming-server-options).

%figure "Streaming server options"

| Key                | Value example | Description                                      |
| ------------------ | ------------- | ------------------------------------------------ |
| port               | 1234          | The port on which the streaming server is open.  |
| mode               | x3d or mjpeg  | The streaming mode used by the streaming server. |
| monitorActivity    |               | Print a dot '.' on stdout every 5 seconds.       |
| disableTextStreams |               | Disable the streaming of stdout and stderr.      |

%end

For example, the following command will start Webots with the streaming server enabled on the TCP port '1234' and will disable the streaming of stdout and stderr: `webots --stream="port:1234;disableTextStreams"`

You can get more information about web streaming in [this section](web-streaming.md).

### Safe Mode

It may happen that Webots cannot start because it is blocked on a world causing a Webots or OpenGL crash.
In this case, Webots can be started in safe mode.
The safe mode forces Webots to start with an empty world, reduces all the OpenGL options, and stores those preferences.
To do this, simply set the environment variable `WEBOTS_SAFE_MODE` in the environment running Webots.

Once successfully started this way, you must unset this environment variable, open again your world and increase [the OpenGL preferences](preferences.md#opengl).
This action may cause a new crash.

#### On Windows

1. Open the `Environment Variables` system dialog box. To do so, look for "environment variable" in the `search bar` of the Windows `start menu`, click on `Edit the system environment variables`, this will open up the `System Properties` dialog box to the `Advanced` tab. Click on the `Environment Variables` button at the bottom.
2. Add a new `WEBOTS_SAFE_MODE` user environment variable. To do so, in the `user variables` panel, click on the `New` button and add a `New User Variable` named `WEBOTS_SAFE_MODE` with a value of `true`.
3. Start Webots as usual.

#### On Linux and macOS

```
export WEBOTS_SAFE_MODE=true
webots
```
