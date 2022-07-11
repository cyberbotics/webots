Ideally, cache should be cleaned every test, and worlds should be changed frequently (don't test everything on the same world)

Test #1:
1. Open (several) complex sample worlds and play (worlds with sounds, worlds with controllers, change textures etc)

Test #2:
1. Create a new project
2. Insert several official Webots PROTO
3. Save, open the world in the text editor and ensure they are declared with a remote url

Test #3:
1. Create a new project
2. Using the PROTO wizard, create a few local PROTO
2.1 Ensure the urls of the declarations and the textures are remote
3. Click add-node button (first time will be slow), close dialog, click add-node again (should be faster)
4. Insert a local PROTO in the world and save
4.1 Ensure the declaration is local
5. Close Webots and manually clear cache (`~/.cache/Cyberbotics/assets`)
6. Start Webots and click add-node and insert the local PROTO
6.1 Ensure the assets are retrieved

Test #4:
1. Open a complex sample world
2. Attempt to modify controller or the world
3. Relocate and ensure everything still works (controller, robot window, sounds, textures)

Test #5:
1. Open complex sample world
2. Right-click proto, two options should be available: `view PROTO source` and `edit PROTO source`
3. When clicking `view PROTO source`, the file should not be editable
4. Click `edit PROTO source`, accept relocation, and SAVE the world.
4.1 Open the world file with text editor, ensure the relocated PROTO is declared locally
4.2 Ensure all urls (EXTERNPROTO & textures) are remote in the local PROTO
4.3 Ensure PROTO changes to the PROTO are enforced when reloading (ex: change `translation IS translation` to ` translation 0 0 5`)
4.4 Rick-click the relocated PROTO, only `edit PROTO source` should be visible


