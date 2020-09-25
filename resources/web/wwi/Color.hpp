#ifndef COLOR_HPP
#define COLOR_HPP
class Color {
public:
  Color(double r, double g, double b);
  bool isColor() const;
  double r() const;
  double g() const;
  double b() const;

  Color convertLinearToSRGB();

private:
  bool mIsColor;
  double mR;
  double mG;
  double mB;

  double linearToSRGB(double c);
};

#endif
