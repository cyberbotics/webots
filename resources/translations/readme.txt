How to translate Webots in your own language?

1. Open wb_generic.ts in Linguist (application provided by Qt)

2. In the appearing dialog box called "Settings for wb_generic", select the source language as POSIX and your own destination language.
   Note: if the dialog message doesn't appear, open it thanks to the "Edit / Translation File Settings..." menu.

3. Save the project as your own file.
   Note: The file name should match this pattern: wb_*.ts

4. Translate the strings. Refer to the Linguist documentation in case of troubles.

5. Call "make" in the current directory to generate the qm file.


---------------------------------------------------------------------------------------------------------------------------------------
Note:

1. Qt internal translation files can be found there: http://qt.gitorious.org/qt/qt/trees/4.8/translations
   The corresponding qm file is generated when typing "make".
   The file name must match (wb_*.ts and qt_*.ts).

2. Don't hesitate to send us your ts files at support@cyberbotics.com
   We'd be glad to add them in the main release.
