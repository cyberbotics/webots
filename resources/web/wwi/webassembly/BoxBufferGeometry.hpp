#ifndef BOXBUFFERGEOMETRY_HPP
#define BOXBUFFERGEOMETRY_HPP

namespace wren {
  class function buildPlane(u, v, w, udir, vdir, width, height, depth) {
  public:
    static BoxBufferGeometry *createBoxBufferGeometry(double width, double height, double depth) {
      return new BoxBufferGeometry(width, height, depth);
    }
    std::map<string, float[72]> attributes();
    std::map<string, int> drawRange();
    void indices(uint16_t out);

  private:
    BoxBufferGeometry(double width, double height, double depth);
    std::map attributes<string, float[72]>;
    std::map drawRange<string, int>;
    uint16_t indices[36];
    int numberOfVertices;
    void buildPlane(char u, char v, char w, int udir, int vdir, double width, double height, double depth);
  };
}  // namespace wren
#endif
