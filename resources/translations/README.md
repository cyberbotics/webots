# How to translate Webots in your own language?

Here is the standard procedure to translate Webots in your own language or improve an existing translation.
If you encounter any difficulty along this path, please contact us on [Discord](https://discordapp.com/invite/nTWbN9m) or at <a href="mailto:support@cyberbotics.com">support@cyberbotics.com</a> and we will be happy to help you.

1. Install the [Webots development environment](https://github.com/cyberbotics/webots/wiki#installation-of-the-webots-development-environment).

2. Start [Linguist](https://doc.qt.io/qt-5/qtlinguist-index.html) from your terminal by typing `linguist` and open `wb_generic.ts`.

3. In the appearing dialog box called "Settings for wb_generic", select the source language as POSIX and your own destination language.
If the dialog message doesn't appear, open it thanks to the "Edit / Translation File Settings..." menu.

4. Save the project as your own file.
The file name should match this pattern: `wb_*.ts`

5. Translate the strings.

6. Call `make` in the current directory to generate the `qm` file.

7. Run Webots to verify that your translation appears properly.

8. Once you completed your translation, please [create a pull request](https://github.com/cyberbotics/webots/blob/master/CONTRIBUTING.md#create-a-pull-request) to submit your new or modified translation files.
