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

#include "WbNodeUtilities.hpp"

#include "../../../include/controller/c/webots/nodes.h"
#include "WbBackground.hpp"
#include "WbBallJoint.hpp"
#include "WbBallJointParameters.hpp"
#include "WbBasicJoint.hpp"
#include "WbBillboard.hpp"
#include "WbBoundingSphere.hpp"
#include "WbBrake.hpp"
#include "WbCamera.hpp"
#include "WbCapsule.hpp"
#include "WbCone.hpp"
#include "WbConnector.hpp"
#include "WbCylinder.hpp"
#include "WbDevice.hpp"
#include "WbElevationGrid.hpp"
#include "WbEmitter.hpp"
#include "WbField.hpp"
#include "WbFluid.hpp"
#include "WbFog.hpp"
#include "WbHinge2Joint.hpp"
#include "WbJointParameters.hpp"
#include "WbLidar.hpp"
#include "WbLinearMotor.hpp"
#include "WbLog.hpp"
#include "WbLogicalDevice.hpp"
#include "WbMFNode.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeReader.hpp"
#include "WbPen.hpp"
#include "WbPlane.hpp"
#include "WbPositionSensor.hpp"
#include "WbProtoModel.hpp"
#include "WbRadar.hpp"
#include "WbReceiver.hpp"
#include "WbRobot.hpp"
#include "WbSelection.hpp"
#include "WbSimulationState.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTemplateManager.hpp"
#include "WbTokenizer.hpp"
#include "WbTouchSensor.hpp"
#include "WbTrack.hpp"
#include "WbTrackWheel.hpp"
#include "WbTransform.hpp"
#include "WbVersion.hpp"
#include "WbViewpoint.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWorld.hpp"

#include <QtCore/QQueue>
#include <QtCore/QStack>
#include <QtCore/QStringList>
#include <cassert>

namespace {
  static void sortNodeListForBackwardCompatibility(QList<WbNode *> &children);
  static QList<WbNode *> getNodeChildrenForBackwardCompatibility(const WbNode *node);
  static QList<WbNode *> getNodeChildrenAndBoundingForBackwardCompatibility(WbNode *node);

  QList<WbNode *> getNodeChildrenForBackwardCompatibility(const WbNode *node) {
    QList<WbNode *> children;
    const WbGroup *const nodeGroup = dynamic_cast<const WbGroup *>(node);
    if (!nodeGroup)
      return children;
    for (int i = 0; i < nodeGroup->childCount(); i++)
      children.append(nodeGroup->child(i));
    return children;
  }

  QList<WbNode *> getNodeChildrenAndBoundingForBackwardCompatibility(WbNode *node) {
    // Make a list of children to be rotated (children, TrackWheel children, bounding object with the group node ignored).
    QList<WbNode *> children = getNodeChildrenForBackwardCompatibility(node);
    QList<WbNode *> newChildren;
    WbNode *boundingObject = static_cast<WbSolid *>(node)->boundingObject();
    for (WbNode *child : children) {
      if (dynamic_cast<WbTrackWheel *>(child))
        newChildren += getNodeChildrenForBackwardCompatibility(child);
      else
        newChildren += child;
    }

    if (!dynamic_cast<WbPose *>(boundingObject) && !dynamic_cast<WbGeometry *>(boundingObject) &&
        !dynamic_cast<WbShape *>(boundingObject) && dynamic_cast<WbGroup *>(boundingObject))
      newChildren += boundingObject->subNodes(false, false);
    else if (boundingObject)
      newChildren.append(boundingObject);

    // Insert the USE nodes in the beginning.
    sortNodeListForBackwardCompatibility(newChildren);

    return newChildren;
  }

  void sortNodeListForBackwardCompatibility(QList<WbNode *> &children) {
    auto getNodeWeight = [](const WbNode *n) {
      // Higher number means higher priority.
      if (dynamic_cast<const WbGeometry *>(n))
        return 3;
      if (n->isDefNode())
        return 0;
      if (n->isUseNode())
        return 2;
      return 1;
    };
    std::sort(children.begin(), children.end(),
              [&getNodeWeight](const WbNode *a, const WbNode *b) -> bool { return getNodeWeight(a) > getNodeWeight(b); });
  }

  bool isAllowedToInsert(const QString &fieldName, const QString &nodeName, const WbNode *node, QString &errorMessage,
                         WbNode::NodeUse nodeUse, const QString &type, bool automaticBoundingObjectCheck = true,
                         bool areSlotAllowed = true) {
    errorMessage = QString();
    const QString defaultErrorMessage =
      QObject::tr("Cannot insert %1 node in '%2' field of %3 node.").arg(nodeName).arg(fieldName).arg(node->nodeModelName());

    if (!areSlotAllowed && nodeName == "Slot") {  // it is not allowed to insert slot->slot-slot
      errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node, because a trio of slot is not allowed.")
                       .arg(nodeName)
                       .arg(fieldName)
                       .arg(node->nodeModelName());
      return false;
    }

    // No robot can be inserted in helix of propellers.
    if (WbNodeUtilities::isRobotTypeName(nodeName) && WbNodeUtilities::isDescendantOfPropeller(node))
      return false;

    if (dynamic_cast<const WbSlot *>(node) && (fieldName == "endPoint")) {  // add something in the endPoint field of a slot
      if (dynamic_cast<const WbSlot *>(node->parentNode())) {  // pair of slots, we can add everything that is allowed in the
                                                               // children field of the parent of first Slot node
        WbNode *parentNode = node->parentNode();
        const WbNode *upperNode = parentNode->parentNode();
        const WbField *upperField = parentNode->parentField(true);
        if (!upperNode || !upperField) {
          assert(false);
          return false;
        }

        return ::isAllowedToInsert(upperField->name(), nodeName, upperNode, errorMessage, nodeUse, type,
                                   automaticBoundingObjectCheck, false);
      }

      // not in a pair => we can only add a Slot of same type
      QString slotType = WbNodeUtilities::slotType(node);
      // if node is a proto and is a parameter of another proto, the pair of slot is not detected
      if (WbNodeReader::current() && node->isProtoInstance() && node->parentNode() && node->parentNode()->isProtoInstance())
        return true;
      if (nodeName != "Slot") {
        errorMessage =
          QObject::tr("Cannot insert %1 node in '%2' field of %3 node: only a slot can be added in the parent slot.")
            .arg(nodeName)
            .arg(fieldName)
            .arg(node->nodeModelName());
        return false;
      }

      bool valid = WbNodeUtilities::isSlotTypeMatch(type, slotType, errorMessage);
      if (!valid)
        errorMessage.prepend(QObject::tr("Cannot insert %1 node in '%2' field of %3 node: ")
                               .arg(nodeName)
                               .arg(fieldName)
                               .arg(node->nodeModelName()));
      return valid;
    }

    const QString &parentModelName = node->nodeModelName();
    bool boundingObjectCase = (nodeUse & WbNode::BOUNDING_OBJECT_USE) || (fieldName == "boundingObject");
    if (automaticBoundingObjectCheck && (nodeUse == WbNode::UNKNOWN_USE)) {
      nodeUse = WbNodeUtilities::checkNodeUse(node);
      boundingObjectCase = boundingObjectCase || (nodeUse & WbNode::BOUNDING_OBJECT_USE);
    }

    const bool childrenField = fieldName == "children";
    const bool isTransformOrTransformDescendant = node->modelName() == "Transform" || WbNodeUtilities::findUpperTransform(node);

    if (childrenField) {
      const bool isInsertingTopLevel = node->isWorldRoot();

      // A robot cannot be a bounding object
      if (!boundingObjectCase && !isTransformOrTransformDescendant && WbNodeUtilities::isRobotTypeName(nodeName) &&
          !WbNodeUtilities::isDescendantOfBillboard(node))
        return true;

      // top level nodes
      bool invalidUseOfTopLevelNode = false;
      if (nodeName == "Fog" || nodeName == "Background") {
        if ((nodeName == "Fog" && WbFog::numberOfFogInstances() != 0) ||
            (nodeName == "Background" && WbBackground::numberOfBackgroundInstances() != 0)) {
          errorMessage = QObject::tr("Cannot insert duplicated %1 node: only one instance is allowed.").arg(nodeName);
          return false;
        } else if (isInsertingTopLevel)
          return true;
        invalidUseOfTopLevelNode = true;

      } else if (nodeName == "WorldInfo" || nodeName == "Viewpoint") {
        if (isInsertingTopLevel) {
          if (WbNodeReader::current())
            return true;

          errorMessage = defaultErrorMessage;
          return false;
        } else
          invalidUseOfTopLevelNode = true;
      }

      if (invalidUseOfTopLevelNode) {
        errorMessage = QObject::tr("%1 node can only be inserted at the top level of the node hierarchy.").arg(nodeName);
        return false;
      }

      if (isInsertingTopLevel) {
        // other nodes that can be inserted at top level
        if (nodeName == "Charger")
          return true;
        if (nodeName == "DirectionalLight")
          return true;
        if (nodeName == "Solid")
          return true;
        if (nodeName == "Fluid")
          return true;
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (nodeName == "Transform")
          return true;
        if (nodeName == "Billboard")
          return true;
        if (nodeName == "Shape")
          return true;
        if (nodeName == "CadShape")
          return true;
        if (nodeName == "PointLight")
          return true;
        if (nodeName == "SpotLight")
          return true;

        errorMessage = QObject::tr("%1 node cannot be inserted at the top level of the node hierarchy.").arg(nodeName);
        return false;
      }
      if (nodeName == "Slot") {
        if (WbNodeUtilities::isDescendantOfBillboard(node))
          return false;
        return !isTransformOrTransformDescendant && !boundingObjectCase;
      }
    }

    static QStringList *fields = NULL;
    if (!fields) {
      fields = new QStringList();
      *fields << "physics"
              << "Physics"
              << "color"
              << "Color"
              << "contactProperties"
              << "ContactProperties"
              << "coord"
              << "Coordinate"
              << "damping"
              << "Damping"
              << "defaultDamping"
              << "Damping"
              << "focus"
              << "Focus"
              << "immersionProperties"
              << "ImmersionProperties"
              << "jointParameters3"
              << "JointParameters"
              << "jointParameters2"
              << "JointParameters"
              << "lens"
              << "Lens"
              << "lensFlare"
              << "LensFlare"
              << "material"
              << "Material"
              << "normal"
              << "Normal"
              << "recognition"
              << "Recognition"
              << "textureTransform"
              << "TextureTransform"
              << "texCoord"
              << "TextureCoordinate"
              << "zoom"
              << "Zoom";
    }

    for (int i = 0, size = fields->size(); i < size; i += 2) {
      if (fieldName == fields->at(i)) {
        if (nodeName == fields->at(i + 1))
          return true;
        else {
          errorMessage = defaultErrorMessage;
          return false;
        }
      }
    }

    if (fieldName == "appearance") {
      if (nodeName == "Appearance")
        return true;
      else if (nodeName == "PBRAppearance") {
        const WbShape *const shape = dynamic_cast<const WbShape *const>(node);
        if (!shape)
          return true;
        const WbGeometry *const geometry = shape->geometry();
        if (!geometry)
          return true;
        if (geometry->nodeType() == WB_NODE_INDEXED_LINE_SET || geometry->nodeType() == WB_NODE_POINT_SET) {
          errorMessage = QObject::tr("The '%1' node doesn't support 'PBRAppearance' in the 'appearance' field of its parent "
                                     "node, please use 'Appearance' instead.")
                           .arg(geometry->nodeModelName());
          return false;
        }
        return true;
      } else {
        errorMessage = defaultErrorMessage;
        return false;
      }
    }

    if (fieldName == "endPoint") {
      if (WbNodeUtilities::isSolidTypeName(nodeName) || nodeName == "SolidReference")
        return true;
      else if (nodeName == "Slot")
        return true;
    } else if (fieldName == "rotatingHead") {
      if (WbNodeUtilities::isSolidTypeName(nodeName))
        return true;
    } else if (fieldName.endsWith("Helix")) {
      if (WbNodeUtilities::isSolidTypeName(nodeName))
        return true;

    } else if (fieldName == "device") {
      const WbJoint *joint = dynamic_cast<const WbJoint *>(node);
      if (parentModelName.startsWith("Hinge") || parentModelName == "Propeller" || parentModelName == "BallJoint") {
        if ((nodeName == "RotationalMotor" &&
             (WbNodeReader::current() || (joint && joint->motor() == NULL) || parentModelName == "Propeller")) ||
            (nodeName == "PositionSensor" && (WbNodeReader::current() || (joint && joint->positionSensor() == NULL))) ||
            (nodeName == "Brake" && (WbNodeReader::current() || (joint && joint->brake() == NULL))))
          return WbNodeUtilities::hasARobotAncestor(node);

      } else if (parentModelName == "SliderJoint" || parentModelName == "Track") {
        const WbTrack *track = dynamic_cast<const WbTrack *>(node);
        if ((nodeName == "LinearMotor" &&
             (WbNodeReader::current() || (joint && joint->motor() == NULL) || (track && track->motor() == NULL))) ||
            (nodeName == "PositionSensor" && (WbNodeReader::current() || (joint && joint->positionSensor() == NULL) ||
                                              (track && track->positionSensor() == NULL))) ||
            (nodeName == "Brake" &&
             (WbNodeReader::current() || (joint && joint->brake() == NULL) || (track && track->brake() == NULL))))
          return WbNodeUtilities::hasARobotAncestor(node);
      }

    } else if (fieldName == "device2") {
      const WbHinge2Joint *joint = dynamic_cast<const WbHinge2Joint *>(node);
      if ((parentModelName == "Hinge2Joint" || parentModelName == "BallJoint") &&
          ((nodeName == "RotationalMotor" && (WbNodeReader::current() || (joint && joint->motor2() == NULL))) ||
           (nodeName == "PositionSensor" && (WbNodeReader::current() || (joint && joint->positionSensor2() == NULL))) ||
           (nodeName == "Brake" && (WbNodeReader::current() || (joint && joint->brake2() == NULL)))))
        return WbNodeUtilities::hasARobotAncestor(node);

    } else if (fieldName == "device3") {
      const WbBallJoint *joint = dynamic_cast<const WbBallJoint *>(node);
      if (parentModelName == "BallJoint" &&
          ((nodeName == "RotationalMotor" && (WbNodeReader::current() || (joint && joint->motor3() == NULL))) ||
           (nodeName == "PositionSensor" && (WbNodeReader::current() || (joint && joint->positionSensor3() == NULL))) ||
           (nodeName == "Brake" && (WbNodeReader::current() || (joint && joint->brake3() == NULL)))))
        return WbNodeUtilities::hasARobotAncestor(node);

    } else if (fieldName == "jointParameters") {
      if (parentModelName == "HingeJoint") {
        if (nodeName == "HingeJointParameters")
          return true;
      } else if (parentModelName == "SliderJoint") {
        if (nodeName == "JointParameters")
          return true;
      } else if (parentModelName == "Hinge2Joint") {
        if (nodeName == "HingeJointParameters")
          return true;
        if ((nodeName == "Hinge2JointParameters") && WbNodeReader::current())
          return true;  // // DEPRECATED, only for backward compatibility
      } else if (parentModelName == "BallJoint") {
        if (nodeName == "BallJointParameters")
          return true;
      }

    } else if (fieldName == "texture" && parentModelName == "Appearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "baseColorMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "roughnessMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "metalnessMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "normalMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "occlusionMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "emissiveColorMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "cubemap" && parentModelName == "Background") {
      return nodeName == "Cubemap";

    } else if (fieldName == "motor" && parentModelName == "Track") {
      return nodeName == "LinearMotor";

    } else if (fieldName == "muscles" && (parentModelName == "LinearMotor" || parentModelName == "RotationalMotor")) {
      QString invalidParentNode;
      if (WbNodeUtilities::findUpperNodeByType(node, WB_NODE_TRACK, 1))
        invalidParentNode = "Track";

      if (!invalidParentNode.isEmpty()) {
        errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node:: "
                                   "%4 node doesn't support Muscle functionality.")
                         .arg(nodeName)
                         .arg(fieldName)
                         .arg(parentModelName)
                         .arg(invalidParentNode);
        return false;
      }
      return nodeName == "Muscle";

    } else if (fieldName == "animatedGeometry" && parentModelName == "Track") {
      return nodeName == "CadShape" || nodeName == "Shape" || nodeName == "Transform" || nodeName == "Pose" ||
             nodeName == "Group" || nodeName == "Slot";

    } else if (fieldName == "bones" && parentModelName == "Skin") {
      return nodeName == "SolidReference";

    } else if (!boundingObjectCase) {
      if (fieldName == "children") {
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (nodeName == "Transform")
          return true;
        if (nodeName == "Shape")
          return true;
        if (nodeName == "CadShape")
          return true;

        if (WbNodeUtilities::isDescendantOfBillboard(node))
          // only Group, Pose, Transform, Shape and CadShape allowed
          return false;

        // if the node itself is a Transform or it has a Transform ancestor exists, prohibit the insertion of Solids,
        // Robots, Devices, Propellers, Lights, and Joints
        if (isTransformOrTransformDescendant) {
          if (nodeName == "PointLight" || nodeName == "SpotLight" || nodeName == "DirectionalLight" ||
              WbNodeUtilities::isSolidTypeName(nodeName) || nodeName == "Propeller" || nodeName.endsWith("Joint"))
            return false;
        }

        if (nodeName == "Solid")
          return true;

        if (WbVrmlNodeUtilities::isFieldDescendant(node, "animatedGeometry"))
          // only Group, Pose, Transform, Shape, CadShape and Slot allowed
          return false;

        if ((parentModelName == "TrackWheel") || WbNodeUtilities::findUpperNodeByType(node, WB_NODE_TRACK_WHEEL))
          // only Group, Pose, Transform, Shape, CadShape and Slot allowed
          return false;

        if (nodeName == "PointLight")
          return true;
        if (nodeName == "SpotLight")
          return true;
        if (nodeName == "Propeller")
          return true;
        if (nodeName == "Charger")
          return true;
        if (nodeName == "Skin" && parentModelName == "Robot")
          return true;
        if (nodeName == "TrackWheel")
          return parentModelName == "Track";

        if (nodeName == "Connector" || nodeName.endsWith("Joint") || nodeName == "VacuumGripper") {
          if (WbNodeUtilities::isSolidTypeName(parentModelName) || WbNodeUtilities::findUpperSolid(node) != NULL)
            return true;

          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node that doesn't have a Solid ancestor.")
                           .arg(nodeName)
                           .arg(fieldName)
                           .arg(parentModelName);
        }

        if (WbNodeUtilities::isSolidDeviceTypeName(nodeName)) {
          if (WbNodeUtilities::hasARobotAncestor(node))
            return true;

          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node that doesn't have a Robot ancestor.")
                           .arg(nodeName)
                           .arg(fieldName)
                           .arg(parentModelName);
        }

      } else if (fieldName == "geometry") {
        if (nodeName == "IndexedLineSet" || nodeName == "PointSet") {
          const WbShape *const shape = dynamic_cast<const WbShape *const>(node);
          if (shape && shape->pbrAppearance()) {
            errorMessage =
              QObject::tr("Can't insert a '%1' node in the 'geometry' field of 'Shape' node if the 'appearance' field "
                          "contains a 'PBRAppearance' node, please use an 'Appearance' node instead.")
                .arg(nodeName);
            return false;
          }
        }
        if (WbNodeUtilities::isGeometryTypeName(nodeName))
          return true;
      }

    } else {  // boundingObject use
      if (fieldName == "boundingObject") {
        if (nodeName == "Shape")
          return true;
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (WbNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;

      } else if (childrenField) {
        if (nodeName == "Shape")
          return true;
        if ((nodeName == "Pose") && (parentModelName != "Pose"))
          return true;
        // if the node is also used outside a boundingObject geometries cannot be inserted directly in the children field
        if (!(nodeUse & WbNode::STRUCTURE_USE) && WbNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;

      } else if (fieldName == "geometry") {
        if (WbNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;
      }

      if (WbNodeUtilities::isGeometryTypeName(nodeName)) {
        errorMessage = QObject::tr("%1 geometry node cannot be used in bounding object.").arg(nodeName);
      } else {
        errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node in bounding object.")
                         .arg(nodeName)
                         .arg(fieldName)
                         .arg(parentModelName);
      }

      return false;
    }

    if (errorMessage.isEmpty())
      errorMessage = defaultErrorMessage;

    return false;
  }

  bool isSolidNode(WbBaseNode *node) {
    return dynamic_cast<WbSolid *>(node);
  }

  bool doesFieldRestrictionAcceptNode(const WbField *const field, const QStringList &nodeNames) {
    assert(field->hasRestrictedValues());
    foreach (const WbVariant variant, field->acceptedValues()) {
      if (variant.type() != WB_SF_NODE)
        continue;
      const WbNode *acceptedNode = variant.toNode();
      assert(acceptedNode);
      if (nodeNames.contains(acceptedNode->modelName()))
        return true;
    }
    return false;
  }
};  // namespace

WbNode *WbNodeUtilities::findUpperNodeByType(const WbNode *node, int nodeType, int searchDegrees) {
  if (node == NULL)
    return NULL;

  int count = searchDegrees > 0 ? searchDegrees : -1;
  WbBaseNode *n = dynamic_cast<WbBaseNode *>(node->parentNode());
  while (n && count != 0) {
    if (n->nodeType() == nodeType)
      return n;
    n = dynamic_cast<WbBaseNode *>(n->parentNode());
    count--;
  }
  return NULL;
}

bool WbNodeUtilities::hasDescendantNodesOfType(const WbNode *node, const QList<int> &nodeTypes) {
  QList<WbNode *> subNodes = node->subNodes(true);
  if (subNodes.isEmpty())
    return false;
  foreach (const WbNode *n, subNodes) {
    if (nodeTypes.contains(dynamic_cast<const WbBaseNode *>(n)->nodeType()))
      return true;
  }
  return false;
}

WbMatter *WbNodeUtilities::findUpperMatter(const WbNode *node) {
  if (node == NULL)
    return NULL;

  WbNode *n = node->parentNode();

  while (n) {
    WbMatter *const matter = dynamic_cast<WbMatter *>(n);
    if (matter)
      return matter;
    else
      n = n->parentNode();
  }
  return NULL;
}

// Note: we assume that a WbSolid cannot be a descendant of a WbFluid and vice versa

WbSolid *WbNodeUtilities::findUpperSolid(const WbNode *node) {
  if (node == NULL)
    return NULL;
  WbMatter *upperMatter = findUpperMatter(node);
  // in the case of slot we want to return the parent node of the first slot
  WbSlot *slot = dynamic_cast<WbSlot *>(upperMatter);
  while (slot) {
    upperMatter = findUpperMatter(upperMatter);
    slot = dynamic_cast<WbSlot *>(upperMatter);
  }
  return dynamic_cast<WbSolid *>(upperMatter);
}

WbPose *WbNodeUtilities::findUppermostPose(const WbNode *node) {
  const WbNode *n = node;
  WbPose *uppermostPose = NULL;
  while (n) {
    const WbPose *pose = dynamic_cast<const WbPose *>(n);
    if (pose)
      uppermostPose = const_cast<WbPose *>(pose);
    n = n->parentNode();
  };
  return uppermostPose;
}

WbSolid *WbNodeUtilities::findUppermostSolid(const WbNode *node) {
  const WbNode *n = node;
  WbSolid *uppermostSolid = NULL;
  while (n) {
    const WbSolid *solid = dynamic_cast<const WbSolid *>(n);
    if (solid)
      uppermostSolid = const_cast<WbSolid *>(solid);
    n = n->parentNode();
  };
  return uppermostSolid;
}

WbMatter *WbNodeUtilities::findUppermostMatter(WbNode *node) {
  const WbNode *n = node;
  WbMatter *uppermostMatter = NULL;
  while (n) {
    const WbMatter *matter = dynamic_cast<const WbMatter *>(n);
    if (matter)
      uppermostMatter = const_cast<WbMatter *>(matter);
    n = n->parentNode();
  };
  return uppermostMatter;
}

WbSolid *WbNodeUtilities::findTopSolid(const WbNode *node) {
  if (node == NULL)
    return NULL;

  const WbNode *n = node;
  const WbNode *parent = n->parentNode();
  WbSolid *topSolid = NULL;
  while (parent) {
    WbSolid *currentSolid = dynamic_cast<WbSolid *>(const_cast<WbNode *>(n));
    if (currentSolid)
      topSolid = currentSolid;

    n = parent;
    parent = n->parentNode();
  }
  return topSolid;
}

WbTransform *WbNodeUtilities::findUpperTransform(const WbNode *node) {
  if (node == NULL)
    return NULL;

  WbNode *n = node->parentNode();
  while (n) {
    WbTransform *const transform = dynamic_cast<WbTransform *>(n);
    if (transform)
      return transform;
    else
      n = n->parentNode();
  }
  return NULL;
}

WbPose *WbNodeUtilities::findUpperPose(const WbNode *node) {
  if (node == NULL)
    return NULL;

  WbNode *n = node->parentNode();
  while (n) {
    WbPose *const pose = dynamic_cast<WbPose *>(n);
    if (pose)
      return pose;
    else
      n = n->parentNode();
  }
  return NULL;
}

bool WbNodeUtilities::hasARobotDescendant(const WbNode *node) {
  const QList<WbNode *> &subNodes = node->subNodes(true);

  foreach (WbNode *const descendantNode, subNodes) {
    if (dynamic_cast<WbRobot *>(descendantNode))
      return true;
  }

  return false;
}

bool WbNodeUtilities::hasADeviceDescendant(const WbNode *node, bool ignoreConnector) {
  const WbGroup *group = dynamic_cast<const WbGroup *>(node);
  if (!group)
    return false;

  const QList<WbNode *> &subNodes = node->subNodes(true);

  foreach (WbNode *const descendantNode, subNodes) {
    if (dynamic_cast<WbDevice *>(descendantNode) && (!ignoreConnector || !dynamic_cast<WbConnector *>(descendantNode)))
      return true;
  }

  return false;
}

bool WbNodeUtilities::hasARobotAncestor(const WbNode *node) {
  WbRobot *robot = findRobotAncestor(node);

  return robot != NULL;
}

WbRobot *WbNodeUtilities::findRobotAncestor(const WbNode *node) {
  if (!node)
    return NULL;

  while (node) {
    if (isRobotTypeName(node->nodeModelName())) {
      const WbRobot *robot = reinterpret_cast<const WbRobot *>(node);
      return const_cast<WbRobot *>(robot);
    }

    node = node->parentNode();
  }
  return NULL;
}

bool WbNodeUtilities::isDescendantOfBillboard(const WbNode *node) {
  if (node == NULL)
    return false;

  WbNode *n = const_cast<WbNode *>(node);
  while (n && !n->isWorldRoot()) {
    WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(n);

    if (!baseNode)
      return false;

    if (baseNode->nodeType() == WB_NODE_BILLBOARD)
      return true;

    n = n->parentNode();
  }

  return false;
}

bool WbNodeUtilities::isDescendantOfPropeller(const WbNode *node) {
  if (node == NULL)
    return false;

  WbNode *n = const_cast<WbNode *>(node);
  while (n && !n->isWorldRoot()) {
    const WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(n);

    if (!baseNode)
      return false;

    if (baseNode->nodeType() == WB_NODE_PROPELLER)
      return true;

    n = n->parentNode();
  }

  return false;
}

WbNode::NodeUse WbNodeUtilities::checkNodeUse(const WbNode *n) {
  WbNode::NodeUse nodeUse = WbNode::UNKNOWN_USE;
  if (n->isDefNode()) {
    // check if at least one of the USE node is in bounding object
    foreach (WbNode *useNode, n->useNodes()) {
      nodeUse = static_cast<WbNode::NodeUse>(nodeUse | checkNodeUse(useNode));
      if (nodeUse == WbNode::BOTH_USE)
        return nodeUse;
    }
  }

  if (n->isProtoParameterNode()) {
    QVector<WbNode *> instances = n->protoParameterNodeInstances();
    // check if at least one of the instances is in bounding object
    foreach (WbNode *instance, instances) {
      nodeUse = static_cast<WbNode::NodeUse>(nodeUse | checkNodeUse(instance));
      if (nodeUse == WbNode::BOTH_USE)
        return nodeUse;
    }
    return nodeUse;
  }

  const WbNode *const p = n->parentNode();
  if (p) {
    const WbMatter *const m = dynamic_cast<const WbMatter *>(p);
    // cppcheck-suppress knownConditionTrueFalse
    if (m)
      return static_cast<WbNode::NodeUse>(nodeUse |
                                          (m->boundingObject() == n ? WbNode::BOUNDING_OBJECT_USE : WbNode::STRUCTURE_USE));

    return static_cast<WbNode::NodeUse>(nodeUse | checkNodeUse(p));
  }

  return static_cast<WbNode::NodeUse>(nodeUse | WbNode::STRUCTURE_USE);
}

bool WbNodeUtilities::isInBoundingObject(const WbNode *node) {
  const WbNode *const p = node->parentNode();
  if (p) {
    const WbMatter *const m = dynamic_cast<const WbMatter *>(p);
    // cppcheck-suppress knownConditionTrueFalse
    if (m) {
      const WbNode *boundingObject = m->boundingObject();

      while (boundingObject) {
        if (boundingObject == node)
          return true;
        boundingObject = boundingObject->protoParameterNode();
      }
    }

    return isInBoundingObject(p);
  }

  return false;
}

WbMatter *WbNodeUtilities::findBoundingObjectAncestor(const WbBaseNode *node) {
  if (!node || !node->isInBoundingObject())
    return NULL;

  WbNode *ancestor = node->parentNode();
  while (ancestor && !ancestor->isWorldRoot()) {
    WbMatter *solidAncestor = dynamic_cast<WbMatter *>(ancestor);
    if (solidAncestor)
      return solidAncestor;
    ancestor = ancestor->parentNode();
  }

  return NULL;
}

bool WbNodeUtilities::isSelected(const WbNode *node) {
  if (!node)
    return false;

  const WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  if (!selectedSolid)
    return false;

  const WbSolid *const upperSolid = findUpperSolid(node);
  const WbSolid *const topSolid = findTopSolid(upperSolid);
  if (upperSolid == selectedSolid || topSolid == selectedSolid)
    return true;

  return false;
}

void WbNodeUtilities::fixBackwardCompatibility(WbNode *node) {
  // We don't want to apply the fix if the node is already >R2021b
  if (!node)
    return;
  if (node->proto() && node->proto()->fileVersion() > WbVersion(2021, 1, 1))
    return;
  const WbNode *const protoAncestor = WbVrmlNodeUtilities::findRootProtoNode(node);
  if (!node->proto() && protoAncestor && protoAncestor->proto()->fileVersion() > WbVersion(2021, 1, 1))
    return;
  if (node->isWorldRoot() && WbTokenizer::worldFileVersion() > WbVersion(2021, 1, 1))
    return;

  static const QString message(
    QObject::tr("Trying to resolve the backwards compability by adjusting the rotation (strategy %1)."));

  // We want to find nodes until PROTOs.
  QList<WbNode *> candidates;
  QQueue<WbNode *> queue;
  QList<WbNode *> subProtos;
  queue.enqueue(node);
  candidates.append(node);
  while (!queue.isEmpty()) {
    const WbNode *const n = queue.dequeue();
    for (WbNode *child : n->subNodes(false, true, true)) {
      if (!child->proto()) {
        queue.append(child);
        if (!candidates.contains(child))
          candidates.append(child);
      } else
        subProtos.append(child);
    }
  }
  sortNodeListForBackwardCompatibility(candidates);

  // Apply rotations to the candidates.
  for (WbNode *candidate : candidates) {
    // This condition is added to handle dangling pointers.
    // TODO: It is very slow though, we may need to improve it.
    if (!node->subNodes(true, true, true).contains(candidate) && candidate != node)
      continue;
    if (dynamic_cast<WbCamera *>(candidate) || dynamic_cast<WbLidar *>(candidate) || dynamic_cast<WbRadar *>(candidate) ||
        dynamic_cast<WbPen *>(candidate) || dynamic_cast<WbEmitter *>(candidate) || dynamic_cast<WbReceiver *>(candidate) ||
        dynamic_cast<WbConnector *>(candidate) || dynamic_cast<WbTouchSensor *>(candidate) ||
        dynamic_cast<WbViewpoint *>(candidate) || dynamic_cast<WbTrack *>(candidate)) {
      // Rotate devices.
      WbMatrix3 rotationFix(-M_PI_2, 0, M_PI_2);
      if (dynamic_cast<WbPen *>(candidate) || dynamic_cast<WbTrack *>(candidate))
        rotationFix = WbMatrix3(-M_PI_2, 0, 0);
      if (dynamic_cast<WbEmitter *>(candidate) || dynamic_cast<WbReceiver *>(candidate) ||
          dynamic_cast<WbConnector *>(candidate) || dynamic_cast<WbTouchSensor *>(candidate))
        rotationFix = WbMatrix3(-M_PI_2, 0, -M_PI_2);

      // Rotate the viewpoint (exception).
      if (dynamic_cast<WbViewpoint *>(candidate)) {
        candidate->info(message.arg("A1"));
        WbViewpoint *const viewpoint = static_cast<WbViewpoint *>(candidate);
        viewpoint->orientation()->setValue(WbRotation(viewpoint->orientation()->value().toMatrix3() * rotationFix));
        viewpoint->save("__init__");
        continue;
      }
      candidate->info(message.arg("A2"));

      // Rotate the device.
      if (candidate != node) {
        WbPose *const device = static_cast<WbPose *>(candidate);
        device->setRotation(WbRotation(device->rotation().toMatrix3() * rotationFix));
        device->save("__init__");
      }

      QList<WbNode *> children = getNodeChildrenAndBoundingForBackwardCompatibility(candidate);
      for (WbNode *child : children) {
        if (!getNodeChildrenAndBoundingForBackwardCompatibility(candidate).contains(child))
          continue;

        WbPose *childPose = dynamic_cast<WbPose *>(child);
        if (childPose) {
          // Squash poses if possible.
          childPose->setRotation(WbRotation(rotationFix.transposed() * childPose->rotation().toMatrix3()));
          childPose->setTranslation(rotationFix.transposed() * childPose->translation());
          childPose->save("__init__");
        } else {
          if (!getNodeChildrenForBackwardCompatibility(candidate).contains(child)) {
            // Child is a bounding object.
            child->info(message.arg("A2_1"));
            WbField *boundingObjectField = candidate->findField("boundingObject");
            if (!boundingObjectField) {
              // field not found if parent is a PROTO and the field is not exposed
              candidate->warn("Conversion to a new Webots format was unsuccessful, please resolve it manually.");
              continue;
            }
            WbPose *const pose = new WbPose();
            pose->setRotation(WbRotation(rotationFix.transposed()));
            pose->save("__init__");
            WbNode *newNode = child->cloneAndReferenceProtoInstance();
            WbNodeOperations::instance()->initNewNode(pose, candidate, boundingObjectField, -1, false, false);
            WbNodeOperations::instance()->initNewNode(newNode, pose, pose->findField("children"), 0, false, false);
          } else {
            // Child is under the `children` field.
            child->info(message.arg("A2_2"));
            WbField *childrenField = candidate->findField("children");
            if (!childrenField) {
              // field not found if parent is a PROTO and the field is not exposed
              candidate->warn("Conversion to a new Webots format was unsuccessful, please resolve it manually.");
              continue;
            }
            WbPose *const pose = new WbPose();
            pose->setRotation(WbRotation(rotationFix.transposed()));
            pose->save("__init__");
            WbNode *newNode = child->cloneAndReferenceProtoInstance();
            WbNodeOperations::instance()->initNewNode(pose, candidate, childrenField, 0, false, false);
            WbNodeOperations::instance()->deleteNode(child);
            WbNodeOperations::instance()->initNewNode(newNode, pose, pose->findField("children"), 0, false, false);
          }
        }
      }
    } else if (dynamic_cast<WbCylinder *>(candidate) || dynamic_cast<WbCapsule *>(candidate) ||
               dynamic_cast<WbCone *>(candidate) || dynamic_cast<WbPlane *>(candidate) ||
               dynamic_cast<WbElevationGrid *>(candidate)) {
      // Rotate geometries.
      const WbMatrix3 rotationFix(-M_PI_2, 0, 0);
      WbNode *const nodeToRotate = dynamic_cast<WbShape *>(candidate->parentNode()) ? candidate->parentNode() : candidate;
      WbNode *const parent = nodeToRotate->parentNode();
      assert(dynamic_cast<WbGroup *>(parent));

      WbPose *const parentPose = dynamic_cast<WbPose *>(parent);
      if (parentPose && parentPose->subNodes(false, false).size() == 1) {
        // Squash poses if possible.
        candidate->info(message.arg("B1"));
        if (dynamic_cast<WbTrackWheel *>(parentPose->parentNode()))
          continue;
        parentPose->setRotation(WbRotation(parentPose->rotation().toMatrix3() * rotationFix));
        parentPose->save("__init__");
      } else
        candidate->warn("Conversion to a new Webots format was unsuccessful, please resolve it manually.");
    }
  }

  // Convert sub-protos.
  for (WbNode *subProto : subProtos) {
    if (subProto->proto() &&
        (subProto->proto()->path().contains(WbStandardPaths::webotsHomePath()) ||
         subProto->proto()->name() == "Bc21bCameraProto") &&
        dynamic_cast<WbPose *>(subProto)) {
      // Since we rotated almost all Webots PROTOs we need to rotate them back.
      // The `Bc21bCameraProto.proto` is added for CI tests (the CI tests are not in the same directory as Webots).

      subProto->info(message.arg("C"));
      const WbMatrix3 rotationFix(-M_PI_2, 0, M_PI_2);
      WbPose *const subProtoPose = static_cast<WbPose *>(subProto);
      subProtoPose->setRotation(WbRotation(subProtoPose->rotation().toMatrix3() * rotationFix));
      subProtoPose->save("__init__");
    } else if (!node->isWorldRoot())
      fixBackwardCompatibility(subProto);
  }
}

WbMatter *WbNodeUtilities::findUpperVisibleMatter(WbNode *node) {
  if (!node)
    return NULL;

  QStack<WbNode *> nodeStack;
  WbNode *parent = node;
  // get node sequence from 'node' to the world root
  while (parent && !parent->isWorldRoot()) {
    while (parent->protoParameterNode())
      parent = parent->protoParameterNode();
    nodeStack.push(parent);
    parent = parent->parentNode();
  }

  if (nodeStack.isEmpty())
    return NULL;

  // iterate back through the stack to check the node visibility
  parent = nodeStack.pop();
  WbMatter *visibleMatter = dynamic_cast<WbMatter *>(parent);
  WbNode *n = NULL;
  while (!nodeStack.isEmpty()) {
    n = nodeStack.pop();
    if (parent->isProtoInstance()) {
      if (!parent->isProtoParameterChild(n))
        return visibleMatter;
    }

    WbMatter *matter = dynamic_cast<WbMatter *>(n);
    if (matter) {
      if (matter->isProtoParameterNode()) {
        WbBaseNode *finalizedInstance = matter->getFirstFinalizedProtoInstance();
        if (finalizedInstance)
          visibleMatter = dynamic_cast<WbMatter *>(finalizedInstance);
      } else
        visibleMatter = matter;
    }
    parent = n;
  }

  return visibleMatter;
}

QList<WbSolid *> WbNodeUtilities::findSolidDescendants(WbNode *node) {
  QList<WbSolid *> solidsList;
  QList<WbNode *> list = findDescendantNodesOfType(node, isSolidNode, false);
  for (int i = 0; i < list.size(); ++i)
    solidsList << dynamic_cast<WbSolid *>(list[i]);
  return solidsList;
}

QList<WbNode *> WbNodeUtilities::findDescendantNodesOfType(WbNode *node, bool (&typeCondition)(WbBaseNode *), bool recursive) {
  QList<WbNode *> result;
  QList<WbNode *> queue;
  QList<WbNode *> visited;  // keep track of visited nodes to avoid infine loops due to SolidReference nodes
  queue.append(node);
  WbNode *n = NULL;
  while (!queue.isEmpty()) {
    n = queue.takeFirst();
    visited.append(n);
    if (typeCondition(dynamic_cast<WbBaseNode *>(n))) {
      result.append(n);
      if (!recursive)
        continue;
    }

    WbGroup *const group = dynamic_cast<WbGroup *>(n);
    if (group) {
      int childCount = group->childCount();
      for (int i = 0; i < childCount; ++i)
        queue.append(group->child(i));
      continue;
    }

    const WbSlot *const slot = dynamic_cast<WbSlot *>(n);
    if (slot) {
      WbNode *baseEndPoint = slot->endPoint();
      if (baseEndPoint && (!slot->solidReferenceEndPoint() || !visited.contains(baseEndPoint)))
        queue.append(baseEndPoint);
      continue;
    }

    const WbBasicJoint *const joint = dynamic_cast<WbBasicJoint *>(n);
    if (joint) {
      WbSolid *endPoint = joint->solidEndPoint();
      if (endPoint && (!joint->solidReference() || !visited.contains(endPoint)))
        queue.append(endPoint);
    }
  }
  if (!result.isEmpty() && result.first() == node)
    result.removeFirst();
  return result;
}

bool WbNodeUtilities::isTemplateRegeneratorField(const WbField *field) {
  const WbField *f = field;
  while (f != NULL) {
    if (f->isTemplateRegenerator() ||
        (f->parentNode() && WbTemplateManager::isNodeChangeTriggeringRegeneration(f->parentNode())))
      return true;
    f = f->parameter();
  }
  return false;
}

bool WbNodeUtilities::isNodeOrAncestorLocked(WbNode *node) {
  WbNode *n = node;
  while (n && !n->isWorldRoot()) {
    WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(n);
    if (baseNode && baseNode->nodeType() == WB_NODE_BILLBOARD)
      return true;

    WbMatter *matter = dynamic_cast<WbMatter *>(n);
    if (matter && matter->isLocked())
      return true;

    n = n->parentNode();
  }

  return false;
}

const WbShape *WbNodeUtilities::findIntersectingShape(const WbRay &ray, double maxDistance, double &distance,
                                                      double minDistance) {
  double timeStep = WbSimulationState::instance()->time();
  const WbGroup *root = WbWorld::instance()->root();
  const int childCount = root->childCount();
  distance = maxDistance;
  const WbShape *shape = NULL;
  WbBoundingSphere *bs;
  for (int i = 0; i < childCount; ++i) {
    bs = root->child(i)->boundingSphere();
    if (bs == NULL)
      continue;
    WbBoundingSphere::IntersectingShape res = bs->computeIntersection(ray, timeStep);
    if (res.shape != NULL && res.distance < distance && res.distance > minDistance) {
      distance = res.distance;
      shape = res.shape;
    }
  }
  return shape;
}

dBodyID WbNodeUtilities::findBodyMerger(const WbNode *node) {
  if (!node)
    return NULL;

  const WbNode *n = node;
  while (n) {
    const WbSolid *s = dynamic_cast<const WbSolid *>(n);
    if (s && s->bodyMerger())
      return s->bodyMerger();
    if (dynamic_cast<const WbBasicJoint *>(n))
      break;
    n = n->parentNode();
  }
  return NULL;
}

bool WbNodeUtilities::isTrackAnimatedGeometry(const WbNode *node) {
  if (node == NULL)
    return false;

  const WbNode *n = node;
  const WbNode *p = n->parentNode();
  while (p) {
    if (dynamic_cast<const WbTrack *>(p) != NULL)
      return (n->parentField() && n->parentField()->model()->name() == "animatedGeometry");
    n = p;
    p = p->parentNode();
  }

  return false;
}

bool WbNodeUtilities::isGeometryTypeName(const QString &modelName) {
  if (modelName == "Cone")
    return true;
  if (modelName == "IndexedLineSet")
    return true;
  if (modelName == "PointSet")
    return true;
  if (isCollisionDetectedGeometryTypeName(modelName))
    return true;
  return false;
}

bool WbNodeUtilities::isCollisionDetectedGeometryTypeName(const QString &modelName) {
  if (modelName == "Box")
    return true;
  if (modelName == "Capsule")
    return true;
  if (modelName == "Cylinder")
    return true;
  if (modelName == "ElevationGrid")
    return true;
  if (modelName == "IndexedFaceSet")
    return true;
  if (modelName == "Mesh")
    return true;
  if (modelName == "Plane")
    return true;
  if (modelName == "Sphere")
    return true;
  return false;
}

bool WbNodeUtilities::isRobotTypeName(const QString &modelName) {
  return modelName == "Robot";
}

static bool isExperimentalDeviceTypeName(const QString &modelName) {
  if (modelName == "Microphone")
    return true;
  if (modelName == "Radio")
    return true;
  return false;
}

bool WbNodeUtilities::isDeviceTypeName(const QString &modelName) {
  if (isSolidDeviceTypeName(modelName))
    return true;
  QStringList deviceTypeName = (QStringList() << "Brake"
                                              << "LinearMotor"
                                              << "PositionSensor"
                                              << "RotationalMotor"
                                              << "Skin");
  return deviceTypeName.contains(modelName);
}

bool WbNodeUtilities::isSolidDeviceTypeName(const QString &modelName) {
  QStringList solidDeviceTypeName = (QStringList() << "Accelerometer"
                                                   << "Altimeter"
                                                   << "Camera"
                                                   << "Compass"
                                                   << "Connector"
                                                   << "Display"
                                                   << "DistanceSensor"
                                                   << "Emitter"
                                                   << "GPS"
                                                   << "Gyro"
                                                   << "InertialUnit"
                                                   << "LED"
                                                   << "Lidar"
                                                   << "LightSensor"
                                                   << "Pen"
                                                   << "Radar"
                                                   << "RangeFinder"
                                                   << "Receiver"
                                                   << "Speaker"
                                                   << "TouchSensor"
                                                   << "Track"
                                                   << "VacuumGripper");
  if (solidDeviceTypeName.contains(modelName))
    return true;

  if (WbNodeReader::current() &&
      isExperimentalDeviceTypeName(modelName)) {  // TODO: this function should not be dependent on the context
    // ensure compatibility with legacy (SWIS/DISAL) worlds while preventing
    // users from creating new Microphone/Radio nodes through WbAddNodeDialog
    return true;
  }
  return false;
}

bool WbNodeUtilities::isSolidTypeName(const QString &modelName) {
  if (modelName == "Solid")
    return true;
  if (modelName == "Charger")
    return true;
  if (WbNodeUtilities::isSolidDeviceTypeName(modelName))
    return true;
  if (isRobotTypeName(modelName))
    return true;

  return false;
}

bool WbNodeUtilities::isMatterTypeName(const QString &modelName) {
  if (modelName == "Fluid")
    return true;
  if (WbNodeUtilities::isSolidTypeName(modelName))
    return true;

  return false;
}

bool WbNodeUtilities::isSlotTypeMatch(const QString &firstType, const QString &secondType, QString &errorMessage) {
  if (firstType.isEmpty() || secondType.isEmpty()) {
    // empty type matches any type
    return true;
  } else if (firstType.endsWith('+') || firstType.endsWith('-')) {  // slots with gender
    if (firstType.left(firstType.size() - 1) == secondType.left(secondType.size() - 1)) {
      if (firstType == secondType) {  // the gender is the same => not compatible
        errorMessage = QObject::tr("the two Slot nodes have the same gender.");
        return false;
      } else  // the gender is different => ok
        return true;
    }
  } else if (firstType == secondType)
    return true;

  // type is not the same
  errorMessage = QObject::tr("types '%1' and '%2' are not matching.").arg(firstType).arg(secondType);
  return false;
}

bool WbNodeUtilities::validateInsertedNode(WbField *field, const WbNode *newNode, const WbNode *parentNode,
                                           bool isInBoundingObject) {
  if (newNode == NULL || field == NULL || parentNode == NULL)
    return true;

  // special case: validation of insertion of Slot node
  // normal validation could fail because the new node is not yet inserted in parent node
  const WbSlot *slot = dynamic_cast<const WbSlot *>(newNode);
  if (slot && slot->endPoint() != NULL) {
    const WbSlot *lowerSlot = slot->slotEndPoint();

    // skip couple of Slot nodes and
    // validate if the endPoint node can be inserted in the parent node
    QList<WbField *> fields = field->internalFields();
    if (fields.isEmpty())
      fields.append(field);
    foreach (WbField *internalField, fields) {
      if (internalField->isParameter())
        // recursive call: check only node field names and not parameter names
        validateInsertedNode(internalField, newNode, internalField->parentNode(), isInBoundingObject);
      else {
        WbNode *internalParentNode = internalField->parentNode();

        // check for single or trio of Slot nodes
        const WbSlot *parentSlot = dynamic_cast<const WbSlot *>(internalParentNode);
        QString errorMessage;
        if (parentSlot && lowerSlot)
          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node, because a trio of slot is not allowed.");
        else if (!parentSlot && !lowerSlot && slot->endPoint())
          errorMessage =
            QObject::tr("Cannot insert %1 node in '%2' field of %3 node: only a slot can be added in the parent slot.");

        if (!errorMessage.isEmpty()) {
          internalParentNode->parsingWarn(
            errorMessage.arg(newNode->modelName()).arg(field->name()).arg(parentNode->nodeModelName()));
          return false;
        }

        if (lowerSlot)
          lowerSlot->validate(internalParentNode, internalField, isInBoundingObject);
        else if (dynamic_cast<const WbSlot *>(internalParentNode)) {
          // upper slot
          WbField *internalParentField = internalParentNode->parentField(true);
          internalParentNode = internalParentNode->parentNode();
          newNode->validate(internalParentNode, internalParentField, isInBoundingObject);
        } else  // invalid structure
          newNode->validate(internalParentNode, internalField, isInBoundingObject);
      }
      return true;
    }
  }

  newNode->validate(NULL, NULL, isInBoundingObject);
  return true;
}

bool WbNodeUtilities::validateExistingChildNode(const WbField *const field, const WbNode *childNode, const WbNode *node,
                                                bool isInBoundingObject, QString &errorMessage) {
  const QString &fieldName = field->name();
  const QString &parentModelName = node->nodeModelName();
  const QString &childModelName = childNode->nodeModelName();

  enum ValidationResultType { NONE = 0, DUPLICATED = 1, ROBOT_ANCESTOR = 2 };
  int result = NONE;
  if (fieldName == "device") {
    const WbJoint *joint = dynamic_cast<const WbJoint *>(node);
    if (parentModelName.startsWith("Hinge") || parentModelName == "BallJoint") {
      if (joint) {
        if (childModelName == "RotationalMotor")
          result = 1 + (static_cast<WbNode *>(joint->motor()) == childNode);
        else if (childModelName == "PositionSensor")
          result = 1 + (static_cast<WbNode *>(joint->positionSensor()) == childNode);
        else if (childModelName == "Brake")
          result = 1 + (static_cast<WbNode *>(joint->brake()) == childNode);
      }
    } else if (parentModelName == "SliderJoint" || parentModelName == "Track") {
      const WbTrack *track = dynamic_cast<const WbTrack *>(node);
      if (childModelName == "LinearMotor")
        result = 1 + (((joint && static_cast<WbNode *>(joint->motor()) == childNode) ||
                       (track && static_cast<WbNode *>(track->motor()) == childNode)));
      else if (childModelName == "PositionSensor")
        result = 1 + (((joint && static_cast<WbNode *>(joint->positionSensor()) == childNode) ||
                       (track && static_cast<WbNode *>(track->positionSensor()) == childNode)));
      else if (childModelName == "Brake")
        result = 1 + (((joint && static_cast<WbNode *>(joint->brake()) == childNode) ||
                       (track && static_cast<WbNode *>(track->brake()) == childNode)));
    } else if (parentModelName == "Propeller" && childModelName == "RotationalMotor")
      result = ROBOT_ANCESTOR;
  } else if (fieldName == "device2") {
    const WbHinge2Joint *joint = dynamic_cast<const WbHinge2Joint *>(node);
    if (joint) {
      if (childModelName == "RotationalMotor")
        result = 1 + (static_cast<WbNode *>(joint->motor2()) == childNode);
      else if (childModelName == "PositionSensor")
        result = 1 + (static_cast<WbNode *>(joint->positionSensor2()) == childNode);
      else if (childModelName == "Brake")
        result = 1 + (static_cast<WbNode *>(joint->brake2()) == childNode);
    }
  } else if (fieldName == "device3") {
    const WbBallJoint *joint = dynamic_cast<const WbBallJoint *>(node);
    if (joint) {
      if (childModelName == "RotationalMotor")
        result = 1 + (static_cast<WbNode *>(joint->motor3()) == childNode);
      else if (childModelName == "PositionSensor")
        result = 1 + (static_cast<WbNode *>(joint->positionSensor3()) == childNode);
      else if (childModelName == "Brake")
        result = 1 + (static_cast<WbNode *>(joint->brake3()) == childNode);
    }
  }
  if (result == ROBOT_ANCESTOR)  // valid if node has a robot ancestor
    return WbNodeUtilities::hasARobotAncestor(node);
  else if (result == DUPLICATED) {  // another device of the same type already exists
    errorMessage = QObject::tr("Only a single %1 node can be inserted in the '%2' field of a %3 node.")
                     .arg(childModelName)
                     .arg(fieldName)
                     .arg(parentModelName);
    return false;
  }

  return ::isAllowedToInsert(fieldName, childModelName, node, errorMessage,
                             isInBoundingObject ? WbNode::BOUNDING_OBJECT_USE : WbNode::STRUCTURE_USE,
                             WbNodeUtilities::slotType(childNode));
}

bool WbNodeUtilities::isAllowedToInsert(const WbField *const field, const QString &nodeName, const WbNode *node,
                                        QString &errorMessage, WbNode::NodeUse nodeUse, const QString &type,
                                        const QStringList &restrictionValidNodeNames, bool automaticBoundingObjectCheck) {
  if (field->hasRestrictedValues() && !doesFieldRestrictionAcceptNode(field, restrictionValidNodeNames))
    return false;
  if (field->isParameter()) {
    bool valid = true;
    foreach (WbField *internalField, field->internalFields()) {
      if (internalField->isParameter())
        // recursive call: check only node field names and not parameter names
        valid = isAllowedToInsert(internalField, nodeName, internalField->parentNode(), errorMessage, WbNode::UNKNOWN_USE, type,
                                  restrictionValidNodeNames, automaticBoundingObjectCheck);
      else {
        const WbNode *parentNode = internalField->parentNode();
        valid = ::isAllowedToInsert(internalField->name(), nodeName, parentNode, errorMessage,
                                    static_cast<const WbBaseNode *>(parentNode)->nodeUse(), type, automaticBoundingObjectCheck);
      }
      if (!valid)
        return false;
    }
    return valid;
  } else
    return ::isAllowedToInsert(field->name(), nodeName, node, errorMessage, nodeUse, type, automaticBoundingObjectCheck);
}

WbNodeUtilities::Answer WbNodeUtilities::isSuitableForTransform(const WbNode *const srcNode, const QString &destModelName,
                                                                int *hasDeviceDescendantFlag) {
  const QString &srcModelName = srcNode->nodeModelName();

  // cannot transform into same type
  if (srcModelName == destModelName)
    return UNSUITABLE;

  WbNode::NodeUse nodeUse = WbNodeUtilities::checkNodeUse(srcNode);
  if (nodeUse & WbNode::BOUNDING_OBJECT_USE) {
    if (srcModelName == "Group" && destModelName == "Pose")
      return SUITABLE;
    if (srcModelName == "Pose" && destModelName == "Group")
      return LOOSING_INFO;

    return UNSUITABLE;
  }

  if (srcModelName == "Group" || srcModelName == "Pose" || srcModelName == "Transform") {
    Answer ok;
    if (srcModelName == "Transform") {
      const WbTransform *transform = dynamic_cast<const WbTransform *>(srcNode);
      ok = transform && transform->scale() != WbVector3(1, 1, 1) ? LOOSING_INFO : SUITABLE;
    } else
      ok = SUITABLE;

    if (destModelName == "Transform" || destModelName == "Pose" || destModelName == "Solid")
      return ok;

    if (destModelName == "Group") {
      const WbPose *p = dynamic_cast<const WbPose *>(srcNode);
      const bool pose = p && (p->translation() != WbVector3(0, 0, 0) || p->rotation().angle() != 0);
      return pose ? LOOSING_INFO : ok;
    }

    if (srcNode->isTopLevel())
      return (destModelName == "Charger" || isRobotTypeName(destModelName)) ? ok : UNSUITABLE;

    if (isSolidDeviceTypeName(destModelName))
      return hasARobotAncestor(srcNode) ? ok : UNSUITABLE;

    return UNSUITABLE;
  }

  if (destModelName == "Group" || destModelName == "Pose" || destModelName == "Transform") {
    if (isSolidTypeName(srcModelName)) {
      bool hasDevices;
      if (hasDeviceDescendantFlag && *hasDeviceDescendantFlag >= 0)  // read cached value
        hasDevices = *hasDeviceDescendantFlag == 1;
      else {
        hasDevices = hasADeviceDescendant(srcNode, true);
        if (hasDeviceDescendantFlag)
          *hasDeviceDescendantFlag = hasDevices ? 1 : 0;
      }

      if (hasDevices)
        return hasARobotAncestor(srcNode->parentNode()) ? LOOSING_INFO : UNSUITABLE;

      const WbSolid *upperSolid = findUpperSolid(srcNode);
      if (!upperSolid && hasAJointDescendant(srcNode))
        return UNSUITABLE;

      return !upperSolid && hasADeviceDescendant(srcNode, false) ? UNSUITABLE : LOOSING_INFO;
    }

    return UNSUITABLE;
  }

  if (isRobotTypeName(srcModelName)) {
    if (destModelName == "Solid" || destModelName == "Charger" || destModelName == "Connector") {
      bool hasDevices;
      if (hasDeviceDescendantFlag && *hasDeviceDescendantFlag >= 0)  // read cached value
        hasDevices = *hasDeviceDescendantFlag == 1;
      else {
        hasDevices = hasADeviceDescendant(srcNode, true);
        if (hasDeviceDescendantFlag)
          *hasDeviceDescendantFlag = hasDevices ? 1 : 0;
      }
      if (destModelName == "Solid" || destModelName == "Charger")
        return hasDevices ? UNSUITABLE : LOOSING_INFO;
      if (destModelName == "Connector") {
        return (hasDevices || !findUpperSolid(srcNode)) ? UNSUITABLE : LOOSING_INFO;
      }
    }

    return UNSUITABLE;
  }

  if (srcModelName == "Charger")
    return destModelName == "Robot" || destModelName == "Solid" || (destModelName == "Connector" && findUpperSolid(srcNode)) ?
             LOOSING_INFO :
             UNSUITABLE;

  if (isSolidTypeName(srcModelName)) {
    if (srcNode->isTopLevel())
      return (destModelName == "Solid" || isRobotTypeName(destModelName) || destModelName == "Charger") ? SUITABLE : UNSUITABLE;

    if (isSolidDeviceTypeName(srcModelName)) {
      if (destModelName == "Solid")
        return LOOSING_INFO;
      if (isSolidDeviceTypeName(destModelName))
        return (srcModelName != "Connector" || hasARobotAncestor(srcNode)) ? LOOSING_INFO : UNSUITABLE;
    } else if (isSolidDeviceTypeName(destModelName))
      return ((destModelName == "Connector" && findUpperSolid(srcNode)) || hasARobotAncestor(srcNode)) ? SUITABLE : UNSUITABLE;

    return UNSUITABLE;
  }

  return UNSUITABLE;
}

QString WbNodeUtilities::slotType(const WbNode *node) {
  const WbSlot *slot = dynamic_cast<const WbSlot *>(node);
  if (slot)
    return slot->slotType();
  else
    return "";
}

bool WbNodeUtilities::isAValidUseableNode(const WbNode *node, QString *warning) {
  WbNode *const n = const_cast<WbNode *>(node);

  const WbSolid *const solid = dynamic_cast<WbSolid *>(n);
  if (solid) {
    if (warning)
      *warning = QObject::tr("Solid nodes cannot be USEd.");
    return false;
  }

  const WbBillboard *const billboard = dynamic_cast<WbBillboard *>(n);
  if (billboard) {
    if (warning)
      *warning = QObject::tr("Billboard nodes cannot be USEd.");
    return false;
  }

  const WbBasicJoint *const joint = dynamic_cast<WbBasicJoint *>(n);
  if (joint) {
    if (warning)
      *warning = QObject::tr("Joint nodes cannot be USEd.");
    return false;
  }

  const WbJointParameters *const jointParameters = dynamic_cast<WbJointParameters *>(n);
  if (jointParameters) {
    if (warning)
      *warning = QObject::tr("JointParameters nodes cannot be USEd.");
    return false;
  }

  const WbTrackWheel *const trackWheel = dynamic_cast<WbTrackWheel *>(n);
  if (trackWheel) {
    if (warning)
      *warning = QObject::tr("TrackWheel nodes cannot be USEd.");
    return false;
  }

  const WbBallJointParameters *const ballJointParameters = dynamic_cast<WbBallJointParameters *>(n);
  if (ballJointParameters) {
    if (warning)
      *warning = QObject::tr("BallJointParameters nodes cannot be USEd.");
    return false;
  }

  const WbLogicalDevice *const logicalDevice = dynamic_cast<WbLogicalDevice *>(n);
  if (logicalDevice) {
    if (warning)
      *warning = QObject::tr("Device nodes cannot be USEd.");
    return false;
  }

  if (hasASolidDescendant(node)) {
    if (warning)
      *warning = QObject::tr("Nodes with a Solid descendant cannot be USEd.");
    return false;
  }

  if (hasAJointDescendant(node)) {
    if (warning)
      *warning = QObject::tr("Nodes with a Joint descendant cannot be USEd.");
    return false;
  }

  return true;
}

bool WbNodeUtilities::hasASolidDescendant(const WbNode *node) {
  if (node == NULL)
    return false;
  WbNode *const n = const_cast<WbNode *>(node);

  const WbSlot *const slot = dynamic_cast<WbSlot *>(n);
  // cppcheck-suppress knownConditionTrueFalse
  if (slot) {
    WbNode *endPoint = slot->endPoint();
    if (endPoint)
      return dynamic_cast<WbSolid *>(endPoint) || hasASolidDescendant(endPoint);
    return false;
  }

  const WbBasicJoint *const joint = dynamic_cast<WbBasicJoint *>(n);
  if (joint)
    return joint->solidEndPoint() != NULL;

  const WbGroup *const group = dynamic_cast<WbGroup *>(n);
  if (group == NULL)
    return false;

  WbMFNode::Iterator it(group->children());
  while (it.hasNext()) {
    WbNode *const next = it.next();
    if (dynamic_cast<WbSolid *>(next) || hasASolidDescendant(next))
      return true;
  }

  return false;
}

bool WbNodeUtilities::hasAJointDescendant(const WbNode *node) {
  if (node == NULL)
    return false;
  WbNode *const n = const_cast<WbNode *>(node);

  const WbSlot *const slot = dynamic_cast<WbSlot *>(n);
  // cppcheck-suppress knownConditionTrueFalse
  if (slot)
    return hasAJointDescendant(slot->endPoint());

  const WbGroup *const group = dynamic_cast<WbGroup *>(n);
  if (group == NULL)
    return false;

  WbMFNode::Iterator it(group->children());
  while (it.hasNext()) {
    WbNode *const next = it.next();
    if (dynamic_cast<WbBasicJoint *>(next) || hasAJointDescendant(next))
      return true;
  }

  return false;
}

////////////////////////////////////////
// Lookup related to DEF name changes //
////////////////////////////////////////

WbBoundingSphere *WbNodeUtilities::boundingSphereAncestor(const WbNode *node) {
  const WbNode *n = node;
  while (n) {
    const WbBaseNode *currentBaseNode = dynamic_cast<const WbBaseNode *>(n);
    if (currentBaseNode && currentBaseNode->boundingSphere()) {
      currentBaseNode->boundingSphere()->recomputeIfNeeded(false);
      if (!currentBaseNode->boundingSphere()->isEmpty()) {
        return currentBaseNode->boundingSphere();
      }
    }
    if (n->isTopLevel())
      break;
    n = n->parentNode();
  }
  return NULL;
}
