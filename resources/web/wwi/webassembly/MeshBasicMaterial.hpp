#ifndef MESHBASICMATERIAL_HPP
#define MESHBASICMATERIAL_HPP

namespace wren {
  class MeshBasicMaterial {
  public:
    static MeshBasicMaterial *createMeshBasicMaterial() { return new MeshBasicMaterial(); }
    bool visible() const;

  private:
    MeshBasicMaterial();
    bool mVisible;
  };
}  // namespace wren
#endif
