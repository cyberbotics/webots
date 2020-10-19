#ifndef MESHBASICMATERIAL_HPP
#define MESHBASICMATERIAL_HPP

namespace wren {
  class MeshBasicMaterial {
  public:
    static MeshBasicMaterial *createMeshBasicMaterial() { return new MeshBasicMaterial(); }
    bool isVisible() const;

  private:
    MeshBasicMaterial();
    bool mIsVisible;
  };
}  // namespace wren
#endif
