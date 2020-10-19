#ifndef COLOR_HPP
#define COLOR_HPP

namespace wren {
  class Color {
  public:
    // Encapsulate memory management
    static Color *createColor(double r, double g, double b) { return new Color(r, g, b); }
    bool isColor() const;
    double r() const;
    double g() const;
    double b() const;

    Color *convertLinearToSRGB();

  private:
    Color(double r, double g, double b);
    bool mIsColor;
    double mR;
    double mG;
    double mB;

    double linearToSRGB(double c);
  };
}  // namespace wren

#endif
