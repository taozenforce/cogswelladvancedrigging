"""
@author: Sergio Sykes - Cogswell Polytechnical College 2014

Example code for a modular rigging system. This code was tested in the creation of goldie and daniel
"""

import maya.cmds as cmds

from advutils import getAttribute, alignObjects, makeControlNode, ROO_XZY, ROO_YXZ


def makePoleVectorLine(startObj, endObj, parent=None):
    curve = cmds.curve(degree=1, point=[(0, 0, 0), (0, 0, 1)], knot=range(2),
                       name='spl_{0}To{1}_poleLine'.format(startObj, endObj))

    curveShape = cmds.listRelatives(curve, s=True, f=True)[0]
    startClu = cmds.cluster(curveShape + '.cv[0]', relative=True,
                            name=curve.replace('spl_', 'clu_', 1) + '_start')[1]
    endClu = cmds.cluster(curveShape + '.cv[1]', relative=True,
                          name=curve.replace('spl_', 'clu_', 1) + '_end')[1]

    cmds.pointConstraint(startObj, startClu)
    cmds.pointConstraint(endObj, endClu)

    for node in curve, startClu, endClu:
        cmds.setAttr(node + '.inheritsTransform', 0)

    cmds.setAttr(curveShape + '.overrideEnabled', 1)
    cmds.setAttr(curveShape + '.overrideDisplayType', 1)
    cmds.setAttr(startClu + '.visibility', 0)
    cmds.setAttr(endClu + '.visibility', 0)

    if parent:
        cmds.parent(startClu, endClu, curve, parent)

    return curve


def makePoleVectorControlFromHandle(name, ikHandle, offset=10, parent=None):
    """
    Creates pole position using the poleVector attribute from the specified ikHandle
    """
    polePosition = [i * offset for i in cmds.getAttr(ikHandle + '.poleVector')[0]]

    ctrl, preTransform = makeControlNode(name)

    # move to start joint location
    cmds.xform(preTransform, worldSpace=True,
               translation=cmds.xform(cmds.ikHandle(ikHandle, q=True, startJoint=True), q=True, worldSpace=True,
                                      translation=True))

    # offset along pole vector (move relative)
    cmds.xform(preTransform, relative=True, objectSpace=True, translation=polePosition)

    midJoint = cmds.ikHandle(ikHandle, q=True, jointList=True)
    curve = makePoleVectorLine(midJoint[len(midJoint) / 2], ctrl, parent)

    cmds.connectAttr(ctrl + '.visibility', curve + '.visibility')

    if parent:
        cmds.parent(preTransform, parent)

    return ctrl, curve


class Rigging(object):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        self._name = name
        self._joints = joints
        self._switchboard = switchboard
        self._mainControl = mainControl
        self._rigControls = dict()
        self.lockAttrs = list()
        self.transform = cmds.group(empty=True, name='grp_{0}_rig'.format(self._name))

        if parent is None:
            locName = 'loc_{0}_parentMe'.format(self._name)
            if cmds.objExists(locName):
                cmds.delete(locName)
            self._parent = cmds.spaceLocator(n=locName)[0]
            alignObjects([self._parent], self._joints[0])
            cmds.parent(self._parent, self.transform)
        else:
            self._parent = parent

        if cmds.objExists('ctl_visibility'):
            visAttr = getAttribute('ctl_visibility', self._name, at='short', min=0, max=1, dv=1)
            cmds.setAttr(visAttr, edit=True, channelBox=True)
            cmds.connectAttr(visAttr, self.transform + '.v')

    def lockAndHide(self, lock):
        for at in self.lockAttrs:
            cmds.setAttr(at, lock=lock)
            cmds.setAttr(at, keyable=not lock)
            cmds.setAttr(at, channelBox=not lock)

    def makeJointSystems(self, prefix, isolation=True, makeConstraints=True):
        joints = cmds.duplicate(self._joints, parentOnly=True, name='{0}_TEMP'.format(prefix))
        joints[0] = cmds.rename(joints[0], self._joints[0].replace('rig_', prefix + '_', 1))

        i = 1
        for jnt, dupe in zip(self._joints[1:], joints[1:]):
            joints[i] = cmds.rename(dupe, jnt.replace('rig_', prefix + '_', 1))
            i += 1

        grp = cmds.group(empty=True, name='grp_{0}_{1}_joints'.format(prefix, self._name))
        root_rotation = cmds.xform(joints[0], q=True, ws=True, rotation=True)
        root_position = cmds.xform(joints[0], q=True, ws=True, translation=True)

        cmds.xform(grp, a=True, ws=True, rotation=root_rotation, translation=root_position)

        if makeConstraints:
            if isolation:
                cmds.pointConstraint(self._joints[0], grp, maintainOffset=False)
                cmds.orientConstraint(self._parent, grp, maintainOffset=True)
            else:
                cmds.parentConstraint()

        cmds.parent(joints[0], grp)

        return joints

    @staticmethod
    def connectJointChains(joints, blendAttr, stretchy=True, useConstraints=False):
        """
        Connect related IK/FK chains to joint chain between baseStartJoint and endJoint
        blendColors nodes are creaed to switch between IK/FK and are conencted to the blendAttr
        if stretchy is True, translationX channels are also blended for later stretchy setups
        isReversed - True changes 0 to fk, 1 to ik
        """

        for jnt in joints:
            fkjoint = jnt.replace('rig_', 'fkj_', 1)
            ikjoint = jnt.replace('rig_', 'ikj_', 1)

            if useConstraints:
                point = cmds.pointConstraint(ikjoint, fkjoint, jnt, mo=False)[0]
                orient = cmds.orientConstraint(ikjoint, fkjoint, jnt, mo=False)[0]

                ikreverse = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_ikfk'.format(jnt))

                orientWeightList = cmds.orientConstraint(orient, q=True, weightAliasList=True)
                pointWeightList = cmds.pointConstraint(point, q=True, weightAliasList=True)

                cmds.connectAttr(blendAttr, '{0}.{1}'.format(orient, orientWeightList[1]))
                cmds.connectAttr(blendAttr, '{0}.{1}'.format(point, pointWeightList[1]))
                cmds.connectAttr(blendAttr, ikreverse + '.inputX')
                cmds.connectAttr(ikreverse + '.outputX', '{0}.{1}'.format(orient, orientWeightList[0]))
                cmds.connectAttr(ikreverse + '.outputX', '{0}.{1}'.format(point, pointWeightList[0]))

                if stretchy:
                    scale = cmds.scaleConstraint(ikjoint, fkjoint, jnt, mo=False)[0]
                    scaleWeightList = cmds.scaleConstraint(point, q=True, weightAliasList=True)
                    cmds.connectAttr(blendAttr, '{0}.{1}'.format(scale, scaleWeightList[1]))
                    cmds.connectAttr(ikreverse + '.outputX', '{0}.{1}'.format(scale, scaleWeightList[0]))

            else:
                rotationblender = cmds.shadingNode('blendColors', asUtility=True, name='bln_{0}_ikfk_rot'.format(jnt))

                cmds.connectAttr(fkjoint + '.rotate', rotationblender + '.color1')
                cmds.connectAttr(ikjoint + '.rotate', rotationblender + '.color2')

                cmds.connectAttr(blendAttr, rotationblender + '.blender')
                cmds.connectAttr(rotationblender + '.output', jnt + '.rotate')

                if stretchy and jnt != joints[0]:
                    node = cmds.shadingNode('blendColors', asUtility=True, name='bln_{0}_ikfk_scale'.format(jnt))

                    cmds.connectAttr(fkjoint + '.tx', node + '.color1R')
                    cmds.connectAttr(ikjoint + '.tx', node + '.color2R')

                    cmds.connectAttr(blendAttr, node + '.blender')
                    cmds.connectAttr(node + '.outputR', jnt + '.tx')

    def makeOrientSwitchNodes(self, joint, preTransform, name=None):
        """
        Creates a orient constraint on the specified preTransform.
        Switches between following the world and following the
        self.parent of the current Rig Module. Joint specifies the which object to apply
        the pointConstraint to the preTransform
        """

        if name is None:
            name = self._name

        cmds.pointConstraint(joint, preTransform)

        parentTarget = cmds.group(empty=True, name='tgt_{0}_local'.format(name))
        worldTarget = cmds.group(empty=True, name='tgt_{0}_world'.format(name))
        alignObjects([parentTarget, worldTarget], preTransform)

        cmds.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        cmds.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        constraint = cmds.orientConstraint(parentTarget, preTransform)[0]
        cmds.orientConstraint(worldTarget, preTransform)
        cmds.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, name + '_isolation',
                                  min=0, max=1, defaultValue=1, keyable=True)

        revAttrNode = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_isolation'.format(name))

        orientWeightList = cmds.orientConstraint(constraint, q=True, weightAliasList=True)
        cmds.connectAttr(orientAttr, '{0}.{1}'.format(constraint, orientWeightList[1]))
        cmds.connectAttr(orientAttr, revAttrNode + '.inputX')
        cmds.connectAttr(revAttrNode + '.outputX', '{0}.{1}'.format(constraint, orientWeightList[0]))

        return [parentTarget, worldTarget]

    def makeNoFlipHelper(self, ikHandle, aimVector):
        helper = cmds.group(empty=True, name=ikHandle.replace('ikh_', 'hlp_ik_') + '_noflipper')
        startJoint = cmds.ikHandle(ikHandle, q=True, sj=True)

        cmds.pointConstraint(startJoint, helper, maintainOffset=False)
        cmds.aimConstraint(ikHandle, helper, aimVector=aimVector, upVector=aimVector, worldUpType='objectrotation',
                           worldUpVector=(0, 1, 0), worldUpObject=self._mainControl)
        return helper


class RiggingLeg(Rigging):
    BALL_ROLL_ATTR_NAME = 'heelRoll'
    TOE_ROLL_ATTR_NAME = 'toeRoll'
    TOE_PIVOT_ATTR_NAME = 'toePivotX'
    TOE_YAW_ATTR_NAME = 'toePivotY'
    HEEL_PIVOT_ATTR_NAME = 'heelPivotX'
    FOOT_BANK_ATTR_NAME = 'footBank'

    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, noFlipVector=None):
        super(RiggingLeg, self).__init__(name, joints, parent, mainControl, switchboard)

        self._noflipvector = noFlipVector
        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(self._switchboard, self._name)
        self._fkjoints = self.makeJointSystems('fkj')
        self._ikjoints = self.makeJointSystems('ikj')
        self.connectJointChains(joints=self._joints[:-1], blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name))

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        fkjointgrp = cmds.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = cmds.listRelatives(self._ikjoints[0], parent=True)[0]

        cmds.parent(fkrig, fkjointgrp, ikrig, ikjointgrp, self.transform)

        self.lockAndHide(True)

    def makeFkRig(self):
        """
        Fk setup for leg
        Assumes user selects hip, knee, ankle, and ball joint; rigs with control curves
        returns created controls as PyNodes
        """
        mainGroup = cmds.group(empty=True, name='grp_fk_{0}_rig'.format(self._name))

        jnts = {'hip': self._fkjoints[0], 'knee': self._fkjoints[1], 'ankle': self._fkjoints[2],
                'ball': self._fkjoints[3], 'toe': self._fkjoints[4]}

        self._rigControls['fk_hip'], hipPreTransform = makeControlNode(name='ctl_fk_{0}_hip'.format(self._name),
                                                                       targetObject=jnts['hip'])
        self._rigControls['fk_knee'], kneePreTransform = makeControlNode(name='ctl_fk_{0}_knee'.format(self._name),
                                                                         targetObject=jnts['knee'])
        self._rigControls['fk_ankle'], anklePreTransform = makeControlNode(name='ctl_fk_{0}_ankle'.format(self._name),
                                                                           targetObject=jnts['ankle'])
        self._rigControls['fk_ball'], ballPreTransform = makeControlNode(name='ctl_fk_{0}_ball'.format(self._name),
                                                                         targetObject=jnts['ball'])

        cmds.pointConstraint(jnts['hip'], hipPreTransform)

        parentTarget = cmds.group(empty=True, name='tgt_fk_{0}_local'.format(self._name))
        worldTarget = cmds.group(empty=True, name='tgt_fk_{0}_world'.format(self._name))
        alignObjects([parentTarget, worldTarget], hipPreTransform)

        cmds.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        cmds.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        constraint = cmds.orientConstraint(parentTarget, hipPreTransform)[0]
        cmds.orientConstraint(worldTarget, hipPreTransform)
        cmds.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, self._name + '_fk_isolation',
                                  min=0, max=1, defaultValue=1, keyable=True)

        revAttrNode = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_fk_isolation'.format(self._name))

        orientWeightList = cmds.orientConstraint(constraint, q=True, weightAliasList=True)
        cmds.connectAttr(orientAttr, '{0}.{1}'.format(constraint, orientWeightList[1]))
        cmds.connectAttr(orientAttr, revAttrNode + '.inputX')
        cmds.connectAttr(revAttrNode + '.outputX', '{0}.{1}'.format(constraint, orientWeightList[0]))

        cmds.parent(parentTarget, worldTarget, mainGroup)

        cmds.orientConstraint(self._rigControls['fk_ball'], jnts['ball'])
        cmds.orientConstraint(self._rigControls['fk_ankle'], jnts['ankle'])
        cmds.orientConstraint(self._rigControls['fk_knee'], jnts['knee'])
        cmds.orientConstraint(self._rigControls['fk_hip'], jnts['hip'])

        cmds.parent(ballPreTransform, self._rigControls['fk_ankle'], absolute=True)
        cmds.parent(anklePreTransform, self._rigControls['fk_knee'], absolute=True)
        cmds.parent(kneePreTransform, self._rigControls['fk_hip'])

        cmds.parent(hipPreTransform, mainGroup)

        if self._switchAttr:
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_ankle'] + '.visibility')
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_ball'] + '.visibility')
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_knee'] + '.visibility')
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_hip'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.visibility')

        return mainGroup

    def makeIkRig(self):
        mainGroup = cmds.group(empty=True, name='grp_ik_{0}_rig'.format(self._name))

        jnts = {'hip': self._ikjoints[0], 'knee': self._ikjoints[1], 'ankle': self._ikjoints[2],
                'ball': self._ikjoints[3], 'toe': self._ikjoints[4]}

        self._rigControls['ik_leg'], legPreTransform = makeControlNode(name='ctl_ik_{0}'.format(self._name),
                                                                       targetObject=jnts['ankle'], alignRotation=False)

        cmds.setAttr(self._rigControls['ik_leg'] + '.rotateOrder', ROO_XZY)

        legHandle = cmds.ikHandle(sj=jnts['hip'], ee=jnts['ankle'], sol='ikRPsolver',
                                  n='ikh_{0}_leg'.format(self._name))[0]
        ballHandle = cmds.ikHandle(sj=jnts['ankle'], ee=jnts['ball'], sol='ikSCsolver',
                                   n='ikh_{0}_ball'.format(self._name))[0]
        toeHandle = cmds.ikHandle(sj=jnts['ball'], ee=jnts['toe'], sol='ikSCsolver',
                                  n='ikh_{0}_toe'.format(self._name))[0]

        self._rigControls['ik_knee'], poleLine = makePoleVectorControlFromHandle('ctl_ik_{0}_pole'.format(self._name),
                                                                                 legHandle, parent=mainGroup)

        cmds.poleVectorConstraint(self._rigControls['ik_knee'], legHandle)
        cmds.parent(legHandle, ballHandle, toeHandle, self._rigControls['ik_leg'])

        # setup reverse foot nodes
        self._rigControls['ik_toe'], toePreTransform = makeControlNode(name='ctl_ik_{0}_toeRoll'.format(self._name),
                                                                       targetObject=jnts['ball'])
        toeRollNode = cmds.group(toePreTransform, name='hlp_ik_{0}_toeRoll'.format(self._name))
        ballRollNode = cmds.group(empty=True, name='hlp_ik_{0}_ballRoll'.format(self._name))
        toePivotNode = cmds.group(empty=True, name='hlp_ik_{0}_toePivot'.format(self._name))
        toeYawNode = cmds.group(empty=True, name='hlp_ik_{0}_toeYaw'.format(self._name))
        heelPivotNode = cmds.group(empty=True, name='hlp_ik_{0}_heelPivot'.format(self._name))
        footBankInNode = cmds.group(empty=True, name='hlp_ik_{0}_footBankIn'.format(self._name))
        footBankOutNode = cmds.group(empty=True, name='hlp_ik_{0}_footBankOut'.format(self._name))

        # positioning
        cmds.xform([ballRollNode, toeYawNode], worldSpace=True,
                   translation=cmds.xform(jnts['ball'], q=True, ws=True, t=True))

        toePivotPosition = cmds.xform(jnts['toe'], q=True, worldSpace=True, translation=True)
        toePivotPosition[1] = 0.0
        cmds.xform(toePivotNode, worldSpace=True, translation=toePivotPosition)

        # Heel pivot and foot bank will vary based on geometry, create a locator that can adjust the pivot
        heelLocator = cmds.spaceLocator(n='loc_ik_{0}_heelPivot'.format(self._name))[0]
        footBankInLocator = cmds.spaceLocator(n='loc_ik_{0}_footBankIn'.format(self._name))[0]
        footBankOutLocator = cmds.spaceLocator(n='loc_ik_{0}_footBankOut'.format(self._name))[0]
        cmds.xform((heelPivotNode, heelLocator, footBankInNode, footBankOutNode, footBankInLocator, footBankOutLocator),
                   worldSpace=True, translation=cmds.xform(jnts['ankle'], q=True, ws=True, t=True))

        cmds.parent(toeRollNode, ballRollNode, toeYawNode)
        cmds.parent(toeYawNode, toePivotNode)
        cmds.parent(toePivotNode, heelPivotNode)
        cmds.parent(heelPivotNode, footBankInNode)
        cmds.parent(footBankInNode, footBankOutNode)
        cmds.parent(footBankOutNode, heelLocator, footBankInLocator, footBankOutLocator, self._rigControls['ik_leg'])

        cmds.makeIdentity((heelLocator, footBankInLocator, footBankOutLocator), apply=True)
        cmds.connectAttr(heelLocator + '.translate', heelPivotNode + '.rotatePivot')
        cmds.connectAttr(footBankInLocator + '.translate', footBankInNode + '.rotatePivot')
        cmds.connectAttr(footBankOutLocator + '.translate', footBankOutNode + '.rotatePivot')

        # Add and connect foot roll attributes
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.TOE_ROLL_ATTR_NAME, softMinValue=-10, softMaxValue=10,
                     defaultValue=0, keyable=True)
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.BALL_ROLL_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.TOE_PIVOT_ATTR_NAME, min=0, max=10, defaultValue=0,
                     keyable=True)
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.TOE_YAW_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.HEEL_PIVOT_ATTR_NAME, min=0, max=10, defaultValue=0,
                     keyable=True)
        cmds.addAttr(self._rigControls['ik_leg'], ln=self.FOOT_BANK_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)

        toeRollMultiply = cmds.shadingNode('multiplyDivide', asUtility=True, n='mul_ik_{0}_toeRoll'.format(self._name))
        ballRollMultiply = cmds.shadingNode('multiplyDivide', asUtility=True,
                                            n='mul_ik_{0}_ballRoll'.format(self._name))
        toePivotMultiply = cmds.shadingNode('multiplyDivide', asUtility=True,
                                            n='mul_ik_{0}_toePivot'.format(self._name))
        toeYawMultiply = cmds.shadingNode('multiplyDivide', asUtility=True, n='mul_ik_{0}_toeYaw'.format(self._name))
        heelPivotMultiply = cmds.shadingNode('multiplyDivide', asUtility=True,
                                             n='mul_ik_{0}_heelPivot'.format(self._name))

        footBankRemap = cmds.shadingNode('remapValue', asUtility=True,
                                         n='rmv_ik_{0}_footBank'.format(self._name))
        footBankClamp = cmds.shadingNode('clamp', asUtility=True,
                                         n='clp_ik_{0}_footBank'.format(self._name))

        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_ROLL_ATTR_NAME),
                         toeRollMultiply + '.input1X')
        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.BALL_ROLL_ATTR_NAME),
                         ballRollMultiply + '.input1X')
        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_PIVOT_ATTR_NAME),
                         toePivotMultiply + '.input1X')
        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_YAW_ATTR_NAME),
                         toeYawMultiply + '.input1Y')
        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.HEEL_PIVOT_ATTR_NAME),
                         heelPivotMultiply + '.input1X')

        cmds.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.FOOT_BANK_ATTR_NAME),
                         footBankRemap + '.inputValue')
        cmds.connectAttr(footBankRemap + '.outValue', footBankClamp + '.inputR')
        cmds.connectAttr(footBankRemap + '.outValue', footBankClamp + '.inputG')

        cmds.setAttr(toeRollMultiply + '.input2X', -9.5)
        cmds.setAttr(ballRollMultiply + '.input2X', 9.5)
        cmds.setAttr(toePivotMultiply + '.input2X', 9.5)
        cmds.setAttr(toeYawMultiply + '.input2Y', 9.5)
        cmds.setAttr(heelPivotMultiply + '.input2X', -11.0)
        cmds.setAttr(footBankRemap + '.inputMin', -10.0)
        cmds.setAttr(footBankRemap + '.inputMax', 10.0)
        cmds.setAttr(footBankRemap + '.outputMin', -90.0)
        cmds.setAttr(footBankRemap + '.outputMax', 90.0)
        cmds.setAttr(footBankClamp + '.maxR', 90.0)
        cmds.setAttr(footBankClamp + '.minG', -90.0)

        cmds.connectAttr(toeRollMultiply + '.outputX', toeRollNode + '.rotateX')
        cmds.connectAttr(ballRollMultiply + '.outputX', ballRollNode + '.rotateX')
        cmds.connectAttr(toePivotMultiply + '.outputX', toePivotNode + '.rotateX')
        cmds.connectAttr(toeYawMultiply + '.outputY', toeYawNode + '.rotateY')
        cmds.connectAttr(heelPivotMultiply + '.outputX', heelPivotNode + '.rotateX')
        cmds.connectAttr(footBankClamp + '.outputR', footBankInNode + '.rotateZ')
        cmds.connectAttr(footBankClamp + '.outputG', footBankOutNode + '.rotateZ')

        cmds.parent(toeHandle, self._rigControls['ik_toe'])
        cmds.parent(ballHandle, toeRollNode)
        cmds.parent(legHandle, ballRollNode)

        # parent to a group aligned to ankle joint, that is constrained only to y orientation
        kneeToFootHelper = cmds.group(empty=True, name='hlp_{0}_knee_to_foot'.format(self._name))

        alignObjects([kneeToFootHelper, ], self._rigControls['ik_leg'])
        cmds.parentConstraint(self._rigControls['ik_leg'], kneeToFootHelper, skipRotate=['x', 'z'], maintainOffset=True)

        noFlipHelper = self.makeNoFlipHelper(legHandle, self._noflipvector)
        kneePolePreT = cmds.listRelatives(self._rigControls['ik_knee'], p=True)
        cmds.parent(kneePolePreT, noFlipHelper)

        poleTwistHelper = cmds.group(kneePolePreT, name='hlp_ik_{0}_poletwist'.format(self._name))
        cmds.xform(poleTwistHelper, objectSpace=True, pivots=(0, 0, 0))
        cmds.connectAttr(kneeToFootHelper + '.rotateY', poleTwistHelper + '.rotateY')

        cmds.parent(noFlipHelper, kneeToFootHelper, legPreTransform, mainGroup)

        revIkVis = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))

        cmds.connectAttr(self._switchAttr, revIkVis + '.inputX')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_leg'] + '.visibility')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateX')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateY')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.visibility')

        return mainGroup


class RiggingArm(Rigging):
    FK_CONTROL_ATTR_BASE = 'fkcontrol'
    IK_CONTROL_ATTR_BASE = 'ikcontrol'

    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, noFlipVector=None):
        super(RiggingArm, self).__init__(name, joints, parent, mainControl, switchboard)

        self._noflipvector = noFlipVector
        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(switchboard, self._name)
        self._fkjoints = self.makeJointSystems('fkj')
        self._ikjoints = self.makeJointSystems('ikj')
        self.connectJointChains(joints=self._joints, blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name))

        fkjointgrp = cmds.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = cmds.listRelatives(self._ikjoints[0], parent=True)[0]

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        cmds.parent(fkjointgrp, ikjointgrp, fkrig, ikrig, self.transform)

        cmds.hide((fkjointgrp, ikjointgrp))

        self.lockAndHide(True)

    def makeFkRig(self):
        mainGroup = cmds.group(empty=True, name='grp_fk_{0}_rig'.format(self._name))

        jnts = {'shoulder': self._fkjoints[0], 'elbow': self._fkjoints[1], 'wrist': self._fkjoints[2]}

        self._rigControls['fk_wrist'], wristPreTransform = makeControlNode(name='ctl_fk_{0}_wrist'.format(self._name),
                                                                           targetObject=jnts['wrist'])

        self._rigControls['fk_elbow'], elbowPreTransform = makeControlNode(name='ctl_fk_{0}_elbow'.format(self._name),
                                                                           targetObject=jnts['elbow'])

        self._rigControls['fk_shoulder'], shoulderPreTransform = \
            makeControlNode(name='ctl_fk_{0}_shoulder'.format(self._name), targetObject=jnts['shoulder'])

        self._rigControls['fk_gimbal_wrist'], armGimbalPreTransform = \
            makeControlNode(name='ctl_fk_{0}_wrist_gimbal'.format(self._name), targetObject=jnts['wrist'])

        parentTarget, worldTarget = self.makeOrientSwitchNodes(jnts['shoulder'], shoulderPreTransform,
                                                               name=self._name + '_fk')
        cmds.parent(parentTarget, worldTarget, mainGroup)

        cmds.orientConstraint(self._rigControls['fk_gimbal_wrist'], jnts['wrist'])
        cmds.orientConstraint(self._rigControls['fk_elbow'], jnts['elbow'])
        cmds.orientConstraint(self._rigControls['fk_shoulder'], jnts['shoulder'])

        cmds.parent(self._rigControls['fk_gimbal_wrist'], self._rigControls['fk_wrist'])
        cmds.parent(wristPreTransform, self._rigControls['fk_elbow'], absolute=True)
        cmds.parent(elbowPreTransform, self._rigControls['fk_shoulder'])

        cmds.parent(shoulderPreTransform, mainGroup)

        cmds.addAttr(self._rigControls['fk_wrist'], at='short', ln='showGimbal', min=0, max=1, defaultValue=1,
                     keyable=False, hidden=False)
        cmds.setAttr(self._rigControls['fk_wrist'] + '.showGimbal', edit=True, channelBox=True)

        if self._switchAttr:
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_shoulder'] + '.visibility')
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_elbow'] + '.visibility')
            cmds.connectAttr(self._switchAttr, self._rigControls['fk_wrist'] + '.visibility')

            gimbalVisMultNode = cmds.shadingNode('multiplyDivide', asUtility=True,
                                                 name='mul_fk_{0}_showGimbal'.format(self._name))

            cmds.connectAttr(self._switchAttr, gimbalVisMultNode + '.input1X')
            cmds.connectAttr(self._rigControls['fk_wrist'] + '.showGimbal',
                             gimbalVisMultNode + '.input2X')
            cmds.connectAttr(gimbalVisMultNode + '.outputX', self._rigControls['fk_gimbal_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.visibility')

        cmds.delete(armGimbalPreTransform)

        return mainGroup

    def makeIkRig(self):
        mainGroup = cmds.group(empty=True, name='grp_ik_{0}_rig'.format(self._name))

        jnts = {'shoulder': self._ikjoints[0], 'elbow': self._ikjoints[1], 'wrist': self._ikjoints[2]}

        self._rigControls['ik_wrist'], armPreTransform = makeControlNode(name='ctl_ik_{0}'.format(self._name),
                                                                         targetObject=jnts['wrist'])

        self._rigControls['ik_gimbal_wrist'], armGimbalPreTransform = \
            makeControlNode(name='ctl_ik_{0}_wrist_gimbal'.format(self._name), targetObject=jnts['wrist'])

        handle = cmds.ikHandle(sj=jnts['shoulder'], ee=jnts['wrist'], sol='ikRPsolver',
                               n='ikh_{0}'.format(self._name))[0]

        self._rigControls['ik_elbow'], poleLine = makePoleVectorControlFromHandle('ctl_ik_{0}_pole'.format(self._name),
                                                                                  handle, parent=mainGroup)

        elbowPreT = cmds.listRelatives(self._rigControls['ik_elbow'], parent=True)[0]

        noFlipHelper = self.makeNoFlipHelper(handle, self._noflipvector)
        cmds.parent(elbowPreT, noFlipHelper)

        cmds.poleVectorConstraint(self._rigControls['ik_elbow'], handle)
        cmds.parent(self._rigControls['ik_gimbal_wrist'], self._rigControls['ik_wrist'])
        cmds.parent(handle, self._rigControls['ik_gimbal_wrist'])
        cmds.orientConstraint(self._rigControls['ik_gimbal_wrist'], jnts['wrist'])

        cmds.parent(armPreTransform, noFlipHelper, mainGroup)

        cmds.addAttr(self._rigControls['ik_wrist'], at='byte', ln='showGimbal', min=0, max=1,
                     defaultValue=1, keyable=False, hidden=False)
        cmds.setAttr(self._rigControls['ik_wrist'] + '.showGimbal', edit=True, channelBox=True)

        revIkVis = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))

        cmds.connectAttr(self._switchAttr, revIkVis + '.inputX')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_wrist'] + '.visibility')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_elbow'] + '.visibility')

        gimbalVisMultNode = cmds.shadingNode('multiplyDivide', asUtility=True,
                                             name='mul_ik_{0}_showGimbal'.format(self._name))

        cmds.connectAttr(revIkVis + '.outputX', gimbalVisMultNode + '.input1X')
        cmds.connectAttr(self._rigControls['ik_wrist'] + '.showGimbal', gimbalVisMultNode + '.input2X')
        cmds.connectAttr(gimbalVisMultNode + '.outputX', self._rigControls['ik_gimbal_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.visibility')

        cmds.delete(armGimbalPreTransform)

        return mainGroup


class RiggingSpine(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, spline=None, switchboard=None):
        super(RiggingSpine, self).__init__(name, joints, parent, mainControl, switchboard)
        self._spline = spline

        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(switchboard, self._name)

        # reparent fk joints to seperate pelvic rotation later
        self._fkjoints = self.makeJointSystems('fkj', makeConstraints=False)
        cmds.parent(self._fkjoints[0], self._fkjoints[1])

        self._ikjoints = self.makeJointSystems('ikj', makeConstraints=False)

        fkjointgrp = cmds.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = cmds.listRelatives(self._ikjoints[0], parent=True)[0]

        cmds.parent(fkjointgrp, ikjointgrp, self.transform)

        self.connectJointChains(joints=self._joints, blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name),
                                stretchy=False, useConstraints=True)

        # Create, position, and set rotation order to controls
        self._rigControls['root'], rootCtlPreT = makeControlNode(name='ctl_{0}_root'.format(self._name),
                                                                 targetObject=self._joints[1], alignRotation=False)
        cmds.setAttr(self._rigControls['root'] + '.rotateOrder', ROO_XZY)
        self.lockAttrs.append(self._rigControls['root'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['root'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['root'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['root'] + '.visibility')

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        cmds.parent(fkrig, self._rigControls['root'])
        cmds.parent(ikrig, self._rigControls['root'])
        cmds.parent(rootCtlPreT, self.transform)

        self.lockAndHide(True)

    def makeFkRig(self):
        mainGroup = cmds.group(empty=True, name='grp_fk_{0}_spinerig'.format(self._name))

        rootJoint = self._fkjoints[1]
        previousControl = None
        for jnt in self._fkjoints:
            if previousControl:
                control, preTransform = makeControlNode(name=jnt.replace('fkj_', 'ctl_fk_', 1), targetObject=jnt)

                if jnt == rootJoint:
                    cmds.parent(preTransform, mainGroup)
                    cmds.parentConstraint(control, jnt)
                else:
                    cmds.parent(preTransform, previousControl)
                    cmds.connectAttr(control + '.rotate', jnt + '.rotate')
            else:
                control, preTransform = makeControlNode(name=jnt.replace('fkj_', 'ctl_', 1), targetObject=rootJoint)

                cmds.parent(preTransform, mainGroup)
                cmds.parentConstraint(control, jnt, maintainOffset=True)

            cmds.connectAttr(self._switchAttr, control + '.visibility')

            self.lockAttrs.append(control + '.translateX')
            self.lockAttrs.append(control + '.translateY')
            self.lockAttrs.append(control + '.translateZ')
            self.lockAttrs.append(control + '.scaleX')
            self.lockAttrs.append(control + '.scaleY')
            self.lockAttrs.append(control + '.scaleZ')
            self.lockAttrs.append(control + '.visibility')

            self._rigControls[jnt] = control
            previousControl = control

        return mainGroup

    def makeIkRig(self):
        mainGroup = cmds.group(empty=True, name='grp_ik_{0}_spinerig'.format(self._name))

        if self._spline is None:
            splineIk = cmds.ikHandle(sj=self._ikjoints[0], ee=self._ikjoints[-1], sol='ikSplineSolver',
                                     parentCurve=False, createCurve=True, simplifyCurve=True, numSpans=2,
                                     rootOnCurve=False, n='sik_{0}_spine'.format(self._name))
            self._spline = cmds.rename(splineIk[2], 'spl_{0}_spinerig'.format(self._name))
        else:
            splineIk = cmds.ikHandle(sj=self._ikjoints[0], ee=self._ikjoints[-1], sol='ikSplineSolver',
                                     createCurve=False, rootOnCurve=False, curve=self._spline, parentCurve=False,
                                     n='sik_{0}_spine'.format(self._name))

        splineHandle = splineIk[0]
        cmds.setAttr(splineHandle + '.inheritsTransform', 0)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_lwr_spine'], spineCtl0PreT = makeControlNode(
            name='ctl_ik_{0}_lower_spine'.format(self._name),
            targetObject=self._ikjoints[1], alignRotation=False)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_mid_spine'], spineCtl1PreT = makeControlNode(
            name='ctl_ik_{0}_middle_spine'.format(self._name))

        midpoint = cmds.pointOnCurve('spl_spine', parameter=0.5, turnOnPercentage=True, position=True)
        cmds.xform(spineCtl1PreT, worldSpace=True, translation=midpoint)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_upr_spine'], spineCtl2PreT = makeControlNode(
            name='ctl_ik_{0}_upper_spine'.format(self._name),
            targetObject=self._ikjoints[-1], alignRotation=False)

        cmds.setAttr(self._rigControls['ik_lwr_spine'] + '.rotateOrder', ROO_YXZ)
        cmds.setAttr(self._rigControls['ik_mid_spine'] + '.rotateOrder', ROO_YXZ)
        cmds.setAttr(self._rigControls['ik_upr_spine'] + '.rotateOrder', ROO_YXZ)

        cmds.parent(spineCtl2PreT, self._rigControls['ik_mid_spine'])
        cmds.parent(spineCtl0PreT, spineCtl1PreT, self._rigControls['root'])
        cmds.parentConstraint(self._rigControls['ik_lwr_spine'], self._ikjoints[0], maintainOffset=True)
        cmds.orientConstraint(self._rigControls['ik_upr_spine'], self._ikjoints[-1], maintainOffset=True)

        # connect controls to ik
        # Using self.joints in place of cluster nodes. They're simpler to work with and do the same thing
        clusterJoints = list()
        cmds.select(clear=True)
        clusterJoints.append(
            cmds.joint(position=cmds.xform(self._ikjoints[1], q=True, worldSpace=True, translation=True), radius=4,
                       name='clj_spine0'))
        cmds.select(clear=True)
        clusterJoints.append(
            cmds.joint(position=midpoint, radius=4, name='clj_spine1'))
        cmds.select(clear=True)
        clusterJoints.append(
            cmds.joint(position=cmds.xform(self._ikjoints[-1], q=True, worldSpace=True, translation=True), radius=4,
                       name='clj_spine2'))

        cmds.parent(clusterJoints[0], self._rigControls['ik_lwr_spine'])
        cmds.parent(clusterJoints[1], self._rigControls['ik_mid_spine'])
        cmds.parent(clusterJoints[2], self._rigControls['ik_upr_spine'])

        cmds.skinCluster(clusterJoints, self._spline, maximumInfluences=2)

        cmds.setAttr(splineHandle + '.dTwistControlEnable', 1)

        cmds.setAttr(splineHandle + '.dWorldUpType', 4)
        cmds.setAttr(splineHandle + '.dWorldUpAxis', 0)

        cmds.setAttr(splineHandle + '.dWorldUpVectorX', 0.0)
        cmds.setAttr(splineHandle + '.dWorldUpVectorY', 0.0)
        cmds.setAttr(splineHandle + '.dWorldUpVectorZ', -1.0)
        cmds.setAttr(splineHandle + '.dWorldUpVectorEndX', 0.0)
        cmds.setAttr(splineHandle + '.dWorldUpVectorEndY', 0.0)
        cmds.setAttr(splineHandle + '.dWorldUpVectorEndZ', -1.0)

        cmds.connectAttr(self._rigControls['ik_lwr_spine'] + '.worldMatrix[0]',
                         splineHandle + '.dWorldUpMatrix')

        cmds.connectAttr(self._rigControls['ik_upr_spine'] + '.worldMatrix[0]',
                         splineHandle + '.dWorldUpMatrixEnd')

        cmds.parent(splineHandle, self._spline, mainGroup)

        revIkVis = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))
        cmds.connectAttr(self._switchAttr, revIkVis + '.inputX')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_lwr_spine'] + '.visibility')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_mid_spine'] + '.visibility')
        cmds.connectAttr(revIkVis + '.outputX', self._rigControls['ik_upr_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.visibility')

        return mainGroup


class RiggingFingers(Rigging):
    FINGER_CURL_ATTR_NAME = 'curl'
    FINGER_STRETCH_ATTR_NAME = 'stretch'
    FINGER_VIS_ATTR_NAME = 'extraControls'

    def __init__(self, name, joints, parent=None, mainControl=None, minStretch=-1.5, maxStretch=1.5,
                 reverseStretch=False):
        super(RiggingFingers, self).__init__(name, joints, parent, mainControl)

        self._minStretch = minStretch
        self._maxStretch = maxStretch
        self._reverseStretch = reverseStretch

        handGrp = self.makeRig()

        cmds.parent(handGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        rootTransforms = list()

        for root in self._joints:
            fingers = [root]
            childFingers = cmds.listRelatives(root, children=True, allDescendents=True, type='joint')
            childFingers.reverse()
            fingers.extend(childFingers)

            curlAttr = None
            stretchAttr = None
            visibilityAttr = None

            previousControl = None
            for fng in fingers[:-1]:
                control, preTransform = makeControlNode(name=fng.replace('rig_', 'ctl_', 1), targetObject=fng)

                if previousControl:
                    cmds.parent(preTransform, previousControl)
                    drivenGrp = cmds.group(control, name='hlp_' + control)

                    curlNode = cmds.shadingNode('multiplyDivide', asUtility=True,
                                                n='mul_{0}_{1}_curl'.format(self._name, fng))

                    cmds.setAttr(curlNode + '.input2Z', -11.0)
                    cmds.connectAttr(curlAttr, curlNode + '.input1Z')
                    cmds.connectAttr(curlNode + '.outputZ', drivenGrp + '.rotateZ')

                    stretchRangeNode = cmds.shadingNode('setRange', asUtility=True,
                                                        n='rng_{0}_{1}_stretch'.format(self._name, fng))

                    if self._reverseStretch:
                        revStretchNode = cmds.shadingNode('reverse', asUtility=True,
                                                          n='rev_{0}_{1}_stretch'.format(self._name, fng))
                        cmds.connectAttr(stretchAttr, revStretchNode + '.inputX')
                        cmds.connectAttr(revStretchNode + '.outputX', stretchRangeNode + '.valueX')
                    else:
                        cmds.connectAttr(stretchAttr, stretchRangeNode + '.valueX')

                    cmds.setAttr(stretchRangeNode + '.minX', self._minStretch)
                    cmds.setAttr(stretchRangeNode + '.maxX', self._maxStretch)
                    cmds.setAttr(stretchRangeNode + '.oldMinX', -10.0)
                    cmds.setAttr(stretchRangeNode + '.oldMaxX', 10.0)

                    cmds.connectAttr(stretchRangeNode + '.outValueX', drivenGrp + '.translateX')

                    cmds.connectAttr(visibilityAttr, preTransform + '.visibility')
                else:
                    curlAttr = getAttribute(control, self.FINGER_CURL_ATTR_NAME,
                                            min=-10.0, max=10.0, defaultValue=0, keyable=True)
                    stretchAttr = getAttribute(control, self.FINGER_STRETCH_ATTR_NAME,
                                               min=-10.0, max=10.0, defaultValue=0, keyable=True)
                    visibilityAttr = getAttribute(control, self.FINGER_VIS_ATTR_NAME, at='short',
                                                  min=0, max=1, defaultValue=1, keyable=True, hidden=False)

                    cmds.setAttr(visibilityAttr, edit=True, channelBox=True)

                cmds.parentConstraint(control, fng)

                if fng != root:
                    self.lockAttrs.append(control + '.rotateX')
                    self.lockAttrs.append(control + '.rotateY')
                else:
                    rootTransforms.append(preTransform)

                self.lockAttrs.append(control + '.translateX')
                self.lockAttrs.append(control + '.translateY')
                self.lockAttrs.append(control + '.translateZ')
                self.lockAttrs.append(control + '.scaleX')
                self.lockAttrs.append(control + '.scaleY')
                self.lockAttrs.append(control + '.scaleZ')
                self.lockAttrs.append(control + '.visibility')

                previousControl = control
                self._rigControls[fng] = control

        groupNode = cmds.group(empty=True, name='grp_{0}_fingers'.format(self._name))
        cmds.parentConstraint(self._parent, groupNode, maintainOffset=False)
        cmds.parent(rootTransforms, groupNode)

        return groupNode


class RiggingHead(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        super(RiggingHead, self).__init__(name, joints, parent, mainControl, switchboard)

        handGrp = self.makeRig()

        cmds.parent(handGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        mainGroup = cmds.group(empty=True, name='grp_{0}_headrig'.format(self._name))

        jnts = {'neck': self._joints[0], 'head': self._joints[1]}

        self._rigControls['neck'], preTransform = makeControlNode(name='ctl_{0}_neck'.format(self._name),
                                                                  targetObject=jnts['neck'])
        self._rigControls['head'], preHeadTransform = makeControlNode(name='ctl_{0}_head'.format(self._name),
                                                                      targetObject=jnts['head'])

        parentTarget = cmds.group(empty=True, name='tgt_{0}_local'.format(self._name))
        worldTarget = cmds.group(empty=True, name='tgt_{0}_world'.format(self._name))
        alignObjects([parentTarget, worldTarget], preTransform)

        cmds.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        cmds.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        cmds.pointConstraint(jnts['neck'], preTransform, maintainOffset=False)

        constraint = cmds.orientConstraint(parentTarget, preTransform,
                                           n=self._name + '_orientConstraint')[0]
        cmds.orientConstraint(worldTarget, preTransform, n=self._name + '_orientConstraint')
        cmds.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, self._name + '_isolation',
                                  min=0, max=1, defaultValue=0, keyable=True)

        revAttrNode = cmds.shadingNode('reverse', asUtility=True, name='rev_{0}_isolation'.format(self._name))

        orientWeightList = cmds.orientConstraint(constraint, q=True, weightAliasList=True)
        cmds.connectAttr(orientAttr, '{0}.{1}'.format(constraint, orientWeightList[1]))
        cmds.connectAttr(orientAttr, revAttrNode + '.inputX')
        cmds.connectAttr(revAttrNode + '.outputX', '{0}.{1}'.format(constraint, orientWeightList[0]))

        cmds.parent([parentTarget, worldTarget], mainGroup)

        cmds.parent(preHeadTransform, self._rigControls['neck'])
        cmds.orientConstraint(self._rigControls['neck'], jnts['neck'])
        cmds.connectAttr(self._rigControls['head'] + '.rotate', jnts['head'] + '.rotate')

        self.lockAttrs.append(self._rigControls['neck'] + '.translateX')
        self.lockAttrs.append(self._rigControls['neck'] + '.translateY')
        self.lockAttrs.append(self._rigControls['neck'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['neck'] + '.visibility')

        self.lockAttrs.append(self._rigControls['head'] + '.translateX')
        self.lockAttrs.append(self._rigControls['head'] + '.translateY')
        self.lockAttrs.append(self._rigControls['head'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['head'] + '.visibility')

        cmds.parent(preTransform, mainGroup)

        return mainGroup


class RiggingClavicle(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        super(RiggingClavicle, self).__init__(name, joints, parent, mainControl, switchboard)

        clavGrp = self.makeRig()

        cmds.parent(clavGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        jnts = {'clav': self._joints[0]}

        self._rigControls['clav'], preTransform = makeControlNode(name='ctl_{0}'.format(self._name),
                                                                  targetObject=jnts['clav'])

        cmds.connectAttr(self._rigControls['clav'] + '.rotate', jnts['clav'] + '.rotate')
        cmds.parentConstraint(self._parent, preTransform, maintainOffset=True)

        self.lockAttrs.append(self._rigControls['clav'] + '.translateX')
        self.lockAttrs.append(self._rigControls['clav'] + '.translateY')
        self.lockAttrs.append(self._rigControls['clav'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['clav'] + '.visibility')

        return preTransform


class RiggingGenericFK(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, isolation=False):
        super(RiggingGenericFK, self).__init__(name, joints, parent, mainControl, switchboard)
        self._isolation = isolation
        fkGrp = self.makeRig()

        cmds.parent(fkGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        allPreTransforms = list()
        for i, jnt in enumerate(self._joints):
            self._rigControls[i], preTransform = makeControlNode(name='ctl_{0}'.format(self._name), targetObject=jnt)

            cmds.connectAttr(self._rigControls[i] + '.rotate', jnt + '.rotate')

            self.lockAttrs.append(self._rigControls[i] + '.translateX')
            self.lockAttrs.append(self._rigControls[i] + '.translateY')
            self.lockAttrs.append(self._rigControls[i] + '.translateZ')
            self.lockAttrs.append(self._rigControls[i] + '.scaleX')
            self.lockAttrs.append(self._rigControls[i] + '.scaleY')
            self.lockAttrs.append(self._rigControls[i] + '.scaleZ')
            self.lockAttrs.append(self._rigControls[i] + '.visibility')

            allPreTransforms.append(preTransform)

        mainGroup = allPreTransforms[0]
        if self._switchboard and self._isolation:
            targetNodes = self.makeOrientSwitchNodes(self._joints[0], mainGroup)
            cmds.parent(targetNodes, mainGroup)
        else:
            cmds.parentConstraint(self._parent, mainGroup, maintainOffset=True)

        return mainGroup


        # CODE SNIPPETS USED TO CREATE GOLDIE & DANIEL

        # import cogbiped
        # reload(cogbiped)

        # from cogbiped import *

        # joints = [u'rig_left_leg_hip', u'rig_left_leg_knee', u'rig_left_leg_ankle', u'rig_left_leg_ball', u'rig_left_leg_toe']
        # RiggingLeg(name='left_leg', parent='rig_spine0', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=[0, -1, 0])

        # joints = [u'rig_right_leg_hip', u'rig_right_leg_knee', u'rig_right_leg_ankle', u'rig_right_leg_ball',
        # u'rig_right_leg_toe']
        # RiggingLeg(name='right_leg', parent='rig_spine0', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=[0, -1, 0])

        # joints = [u'rig_spine0', u'rig_spine1', u'rig_spine2', u'rig_spine3', u'rig_spine4']
        # RiggingSpine(name='spine', parent='ctl_main', joints=joints, mainControl='ctl_main',
        # spline='spl_spine', switchboard='ctl_settings')

        # joints = [u'rig_left_arm_shoulder', u'rig_left_arm_elbow', u'rig_left_arm_wrist']
        # RiggingArm(name='left_arm', parent='rig_left_arm_clavicle', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=(1, 0, 0))

        # joints = [u'rig_right_arm_shoulder', u'rig_right_arm_elbow', u'rig_right_arm_wrist']
        # RiggingArm(name='right_arm', parent='rig_right_arm_clavicle', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=(-1, 0, 0))

        # joints = [u'rig_spine5', u'rig_head']
        # RiggingHead(name='head', parent='rig_spine4', joints=joints, mainControl='ctl_main',
        #             switchboard='ctl_settings')

        # joints = [u'rig_left_fng_thumb0', u'rig_left_fng_index0', u'rig_left_fng_middle0', u'rig_left_fng_pinky0']
        # RiggingFingers(name='left_hand', parent='rig_left_arm_wrist', joints=joints, mainControl='ctl_main',
        #                minStretch=-0.1, maxStretch=0.1)

        # joints = [u'rig_right_fng_thumb0', u'rig_right_fng_index0', u'rig_right_fng_middle0', u'rig_right_fng_pinky0']
        # RiggingFingers(name='right_hand', parent='rig_right_arm_wrist', joints=joints, mainControl='ctl_main',
        #                minStretch=-0.1, maxStretch=0.1, reverseStretch=True)

        # joints = [u'rig_left_arm_clavicle']
        # RiggingClavicle(name='left_clav', parent='rig_spine4', joints=joints, mainControl='ctl_main')

        # joints = [u'rig_right_arm_clavicle']
        # RiggingClavicle(name='right_clav', parent='rig_spine4', joints=joints, mainControl='ctl_main')