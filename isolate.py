import pymel.core as pmc

def getAttribute(node, attr, **kwargs):
    if not pmc.attributeQuery(attr, node=node, exists=True):
        pmc.addAttr(node, ln=attr, **kwargs)

    return pmc.Attribute('{0}.{1}'.format(node, attr))

def isolateOnControl(worldTarget, localTarget, transform, control, attributeName,
    orientation=True, position=False, useLocators=False):

    if isinstance(worldTarget, basestring):
        worldTarget = pmc.PyNode(worldTarget)

    if isinstance(localTarget, basestring):
        localTarget = pmc.PyNode(localTarget)

    if isinstance(transform, basestring):
        transform = pmc.PyNode(transform)

    basename = transform.shortName()

    worldGrp = pmc.createNode('transform', name='tgt_{0}_to_world'.format(basename))
    worldGrp.setRotation(transform.getRotation('world'), 'world')
    worldGrp.setTranslation(transform.getTranslation('world'), 'world')

    localGrp = pmc.createNode('transform', name='tgt_{0}_to_local'.format(basename))
    localGrp.setRotation(transform.getRotation('world'), 'world')
    localGrp.setTranslation(transform.getTranslation('world'), 'world')

    pmc.parentConstraint(worldTarget, worldGrp, maintainOffset=True)
    pmc.parentConstraint(localTarget, localGrp, maintainOffset=True)

    constraints = list()
    if orientation:
        con = pmc.orientConstraint(worldGrp, localGrp, transform, maintainOffset=False)
        con.interpType.set(2)
        constraints.append(con)

    if position:
        con = pmc.pointConstraint(worldGrp, localGrp, transform, maintainOffset=False)
        constraints.append(con)

    isolateAttr = getAttribute(control, attributeName, min=0, max=1, keyable=True)

    for con in constraints:
        weightAttrs = con.getWeightAliasList()        
        rev = pmc.createNode('reverse', name='rev_{0}_to_local'.format(basename))
        isolateAttr.connect(weightAttrs[0])
        isolateAttr.connect(rev.inputX)
        rev.outputX.connect(weightAttrs[1])

