import pymel.core as pmc

ROO_XYZ, ROO_YZX, ROO_ZXY, ROO_XZY, ROO_YXZ, ROO_ZYX = range(6)


def getAttribute(node, attr, **kwargs):
    if not pmc.attributeQuery(attr, node=node, exists=True):
        pmc.addAttr(node, ln=attr, **kwargs)

    return pmc.Attribute('{0:s}.{1:s}'.format(node, attr))


def alignObjects(sources, target, position=True, rotation=True, rotateOrder=False, viaRotatePivot=False):
    """
    Aligns list of sources to match target
    If target has a different rotation order,
    sources rotation order will be set to that of the target
    """

    rotateOrderXYZ = pmc.getAttr(target + '.rotateOrder')

    if viaRotatePivot:
        targetPos = pmc.xform(target, q=True, worldSpace=True, rotatePivot=True)
    else:
        targetPos = pmc.xform(target, q=True, worldSpace=True, translation=True)

    if rotation and isinstance(target, pmc.nodetypes.Joint):
        # Use temporary locator in case we're aligning to joints
        # xform gives inconsistent results for them
        tmpLoc = pmc.spaceLocator()
        pmc.setAttr(tmpLoc + '.rotateOrder', rotateOrderXYZ)
        tmpConstraint = pmc.orientConstraint(target, tmpLoc, maintainOffset=False)
        targetRot = pmc.xform(tmpLoc, q=True, worldSpace=True, rotation=True)

        pmc.delete(tmpConstraint, tmpLoc)
    else:
        targetRot = pmc.xform(target, q=True, worldSpace=True, rotation=True)

    if isinstance(sources, (str, pmc.PyNode)):
        sources = [sources]

    for src in sources:
        if rotateOrder:
            pmc.setAttr(src + '.rotateOrder', rotateOrderXYZ)

        if position:
            pmc.xform(src, worldSpace=True, translation=targetPos)

        if rotation:
            pmc.xform(src, worldSpace=True, rotation=targetRot)


def makeControlNode(name, targetObject=None, alignRotation=True):
    control = pmc.group(empty=True, name=name)
    pretransform = pmc.group(empty=True, name='pre_' + control)

    if targetObject:
        alignObjects([control, pretransform], targetObject, rotation=alignRotation)

    pmc.parent(control, pretransform)
    pmc.makeIdentity(control, apply=True)

    return control, pretransform


def zeroOut(node, prefix='pre'):
    node = pmc.PyNode(node)
    preTransform = pmc.createNode('transform', n='{0}_{1}'.format(prefix, node))
    alignObjects([preTransform, ], node)

    parent = node.firstParent2()
    if parent:
        pmc.parent(preTransform, parent)

    pmc.parent(node, preTransform)
    #pmc.makeIdentity(node, apply=True)

    return preTransform


def makeDuplicateJoints(joints, search, replace, connectBone=True):
    """
    If connectBone is true, parent joints to each other based on selection order
    """

    dupeJoints = list()
    for jnt in joints:
        dupe = pmc.createNode('joint', n=jnt.shortName().replace(search, replace, 1))
        alignObjects(dupe, jnt, rotateOrder=True)

        if connectBone and len(dupeJoints):
            dupe.setParent(dupeJoints[-1])

        pmc.makeIdentity(dupe, apply=True)
        dupe.radius.set(1.0)
        dupe.overrideEnabled.set(1)
        dupe.overrideColor.set(17)
        dupeJoints.append(dupe)

    return dupeJoints