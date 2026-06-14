#include <util/util.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using namespace korsim::util;

class TfTestNode3D : public Node3D{
	GDCLASS(TfTestNode3D, Node3D)

protected:
	static void _bind_methods();

public:
	TfTestNode3D(){};
	~TfTestNode3D(){};

    void setPosAero(godot::Vector3 val);
    godot::Vector3 getPosAero() const;
    void setEulerAero(godot::Vector3 val);
    godot::Vector3 getEulerAero() const;
};
