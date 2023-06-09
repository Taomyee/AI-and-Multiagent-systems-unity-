﻿# Soccer assignment

The soccer environments are built using network code. If you wish to ignore most of that, you can build your AI code into the [CarAISoccer](Assets/Scripts/Soccer/CarAISoccer.cs) and [DroneAISoccer](Assets/Scripts/Soccer/DroneAISoccer.cs) classes respectively. 

It is possible to run the environment in one two ways:

- Host
- Client / Server

## Host

In the host mode, both the clients and the server are run inside one instance of the environment.
This can be started by running the environment with "Play" and clicking on the "Host" button in the editor.

Or alternatively with the following command (assuming an executable named "soccer_cars"):
```
./Soccer_cars -mode host
```

The host mode can be a convenient mode for testing changes quickly. 

You can change the code in the GameManager to launch into host mode directly. **Dont forget to change it back when its competition time!**

## Client / Server

In this mode, 3 instances of the application are necessary. Therefore, a built artifact is required to launch it in this manner.

Once you have the executable, you can either run it 3 times and use the GUI to start 1 server and 2 clients. The server should be started first. 

Or alternatively using this command (assuming an executable named "soccer_cars"):
```
 ./Soccer_cars -mode server & ./Soccer_cars -mode client & ./Soccer_cars -mode client
```
Delays in the Client/Server mode are larger than in host mode. It is suggested you test your algorithms out in both modes.
Note that the **final competitions will be in Client/Server mode**, to enable you to share executables and run games against other teams without seeing their code.

# Building an executable

![alt text](build1.png)

It is best to build a single scene into the executable at once, unless you know what you are doing.

![alt text](build2.png)

Once the executable is built, you can run it directly  as any other application and use the UI as in the editor or using the commands in the sections above.

You can change the Target Platform in order to build versions that can be used by people on other plaforms.
In order for that to be available you have to add the corresponding modules to the version of the engine you are using (see below).

![alt text](build3.png)

NB! When building for different platforms, do not open or launch any of the files!
For example, a Mac executable is a Folder in Windows.
If you navigate it, it will not work properly in Mac as Windows will change some of the properties.

![alt text](build4.png)

NB! On some platforms you might need to delete your old build artifacts if you wish to build an artifact into the same folder as a previous one to see any changes.

## Debugging executables

I suggest reading the Unity [netcode for gameobjects](https://docs-multiplayer.unity3d.com/netcode/current/about) manual in general, if you wish to understand more about how this environment is built.

At the very least, I suggest looking at the information about [debugging.](https://docs-multiplayer.unity3d.com/netcode/current/tutorials/testing/techniques_and_tricks_for_debugging_multiplayer_games)

If you are working on Windows, I suggest you check out this [subsection about logfiles](https://docs-multiplayer.unity3d.com/netcode/current/tutorials/command-line-helper#test-on-windows) and if you are on Mac, [subsection about launching in general](https://docs-multiplayer.unity3d.com/netcode/current/tutorials/command-line-helper#test-on-macos).
