import pymel.core as pmc


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