Ideally, cache should be cleaned every test, and worlds should be changed frequently (don't test everything on the same world)

Test #1:
1. Open (several) complex sample worlds and play (worlds with sounds, worlds with controllers, change textures etc)

Test #2:
1. Create a new project
2. Insert several official Webots PROTO
3. Save, open the world in the text editor and ensure they are declared with a remote url


Test #3:
1.

Test #4:
1. Open complex sample world
2. Right-click proto, two options should be available: `view PROTO source` and `edit PROTO source`
3. When clicking `view PROTO source`, the file should not be editable
4. Click `edit PROTO source`, accept relocation, and SAVE the world.
4.1 Open the world file with text editor, ensure the relocated PROTO is declared locally
4.2 Ensure all urls (EXTERNPROTO & textures) are remote in the local PROTO
4.3 Ensure PROTO changes to the PROTO are enforced when reloading (ex: change `translation IS translation` to ` translation 0 0 5`)
4.4 Rick-click the relocated PROTO, only `edit PROTO source` should be visible


