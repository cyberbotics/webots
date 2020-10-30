## Citing Webots

If you write a scientific paper or describe your project involving Webots on a web page, we would greatly appreciate it if you could add a reference to Webots.
For example by explicitly mentioning Cyberbotics' website or by referencing a journal paper that describes Webots.
To make this simpler, we provide here some citation examples, including BibTex entries that you can use in your own documents.

### Citing Cyberbotics' Website

*This project uses [Webots](http://www.cyberbotics.com), an open-source mobile robot simulation software developed by Cyberbotics Ltd.*

*This project uses Webots (http://www.cyberbotics.com), an open-source mobile robot simulation software developed by Cyberbotics Ltd.*

The BibTex reference entry may look odd, as it is very different from a standard paper citation and we want the specified fields to appear in the normal plain citation mode of LaTeX.

```tex
@MISC{Webots,
  AUTHOR = {Webots},
  TITLE  = {http://www.cyberbotics.com},
  NOTE   = {Open-source Mobile Robot Simulation Software},
  EDITOR = {Cyberbotics Ltd.},
  URL    = {http://www.cyberbotics.com}
}
```

Once compiled with LaTeX, it should display as follows:

*References*

```
[1] Webots.
http://www.cyberbotics.com.
Open-source Mobile Robot Simulation Software.
```

### Citing a Reference Journal Paper about Webots

A reference paper was published in the International Journal of Advanced Robotics Systems.
Here is the BibTex entry:

```tex
@ARTICLE{Webots04,
  AUTHOR  = {Michel, O.},
  TITLE   = {Webots: Professional Mobile Robot Simulation},
  JOURNAL = {Journal of Advanced Robotics Systems},
  YEAR    = {2004},
  VOLUME  = {1},
  NUMBER  = {1},
  PAGES   = {39--42},
  URL     = {http://www.ars-journal.com/International-Journal-of-
             Advanced-Robotic-Systems/Volume-1/39-42.pdf}
}
```
