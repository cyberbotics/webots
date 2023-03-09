// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  This controller gives to its node the following behavior:
 *               Listen the keyboard. According to the pressed key, send a
 *               message through an emitter or handle the position of Robot1
 */

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Node;

public class Driver extends Supervisor {

  private final int timeStep = 128;

  private Emitter emitter;
  private Field translationField;
  private Keyboard keyboard;
  private double x = -0.3;
  private double y = -0.1;
  private double[] translation = {x, y, 0};

  public Driver() {
    emitter = getEmitter("emitter");
    Node robot = getFromDef("ROBOT1");
    if (robot == null)
      // robot might be null if the controller is about to quit
      System.exit(1);

    translationField = robot.getField("translation");
    keyboard = getKeyboard();
    keyboard.enable(timeStep);
  }

  public void run() {

    String previous_message="";
    String message="";

    displayHelp();

    // main loop
    // quit the loop when the simulation is ended
    while (step(timeStep) != -1) {

      // Read sensors; update message according to the pressed keyboard key
      int k=keyboard.getKey();
      switch (k){
        case 'A':
          message = "avoid obstacles";
          break;
        case 'F':
          message = "move forward";
          break;
        case 'S':
          message = "stop";
          break;
        case 'T':
          message = "turn";
          break;
        case ('I'):
          displayHelp();
          break;
        case 'G':{
          double[] translationValues = translationField.getSFVec3f();
          System.out.println("ROBOT1 is located at ("+translationValues[0]+","+translationValues[1]+")");
          break;}
        case 'R':
          System.out.println("Teleport ROBOT1 at ("+x+","+y+")");
          translationField.setSFVec3f(translation);
          break;
        default:
          message = "";
      }

      // send actuators commands; send a new message through the emitter device
      if (!message.equals("") && !message.equals(previous_message)) {
        previous_message=message;
        System.out.println("Please, "+message);
        byte[] formated = message.getBytes();
        emitter.send(formated);
      }
    }
  }

  private void displayHelp(){
    System.out.println("Commands:\n"+
                   " I for displaying the commands\n"+
                   " A for avoid obstacles\n"+
                   " F for move forward\n"+
                   " S for stop\n"+
                   " T for turn\n" +
                   " R for positioning ROBOT1 at (-0.3,-0.1)\n"+
                   " G for knowing the (x,y) position of ROBOT1");
  }

  public static void main(String[] args) {
    Driver controller = new Driver();
    controller.run();
  }
}
