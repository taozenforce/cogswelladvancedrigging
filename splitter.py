# Usage:

# import splitter
# reload(splitter)

# spl = splitter.Splitter()
# spl.GUI()

# izip gives a iterable pair of two lists
# example: izip(p, q) outputs (p[0], q[0]), (p[1], q[1])

from itertools import izip

import maya.cmds as cmds


class Splitter(object):
    WINDOW_NAME = 'JointSplitterCMDS'
    WINDOW_TITLE = 'Joint Splitter CMDS'

    def __init__(self):
        self._divisions = 2
        self._win = None
        self._slider = None

    def GUI(self):
        self._win = cmds.window(self.WINDOW_NAME, title=self.WINDOW_TITLE)
        cmds.columnLayout(adjustableColumn=True)
        self._slider = cmds.intSliderGrp(label='Segments', field=True, min=2, max=100, value=2)
        cmds.button(label='Okay', c=lambda *args: self._callback())
        cmds.button(label='Cancel', c=lambda *args: cmds.deleteUI(self._win))
        cmds.showWindow(self._win)

    def _callback(self):
        joints = cmds.ls(selection=True, type='joint')
        if len(joints) == 0:
            return

        self._divisions = cmds.intSliderGrp(self._slider, q=True, value=True)
        self.doSplit(joints, self._divisions)

    @staticmethod
    def doSplit(joints, divisions):
        for jnt in joints:
            jnt2 = cmds.listRelatives(jnt, type='joint')[0]
            a = cmds.xform(jnt, q=True, ws=True, translation=True)
            b = cmds.xform(jnt2, q=True, ws=True, translation=True)

            # compute distance vector by subtracting b - a 
            # using lists, so we have to loop through each index
            # list comprehensions allow us to cycle through lists in a single line
            # divide by number of sections to get our increment step
            div = [(ii - i) / divisions for i, ii in izip(a, b)]

            # joint command automatically parents joints to what's selected
            # since we may potentially work with multi-joint selection
            # make sure to select only the current joint we're working with
            cmds.select(jnt, replace=True)
            dupe = None
            for i in xrange(1, divisions):
                # find the final position to place current joint by adding our
                # segmented distance vector (div) to the original joint's (a) position
                pos = [p + (q * i) for p, q in izip(a, div)]

                # reset joint orientation to match starting joint
                dupe = cmds.joint(position=pos)
                cmds.makeIdentity(dupe, apply=True, jointOrient=True)

            # reparent joint to last duplicate we made, completing the joint chain
            if dupe:
                cmds.parent(jnt2, dupe)


# import pymel.core as pmc
#
#
# class Splitter(object):
#     WINDOW_NAME = 'JointSplitterPyMel'
#     WINDOW_TITLE = 'Joint Splitter PyMel'
#
#     def __init__(self):
#         self._divisions = 2
#         self._win = None
#         self._slider = None
#
#     def GUI(self):
#         self._win = pmc.window(self.WINDOW_NAME, title=self.WINDOW_TITLE)
#         pmc.columnLayout(adjustableColumn=True)
#         self._slider = pmc.intSliderGrp(label='Segments', field=True, min=2, max=100, value=2)
#         pmc.button(label='Okay', c=pmc.Callback(self._callback))
#         pmc.button(label='Cancel', c=pmc.Callback(pmc.deleteUI, self._win))
#         self._win.show()
#
#     def _callback(self):
#         joints = pmc.ls(selection=True, type='joint')
#         if len(joints) == 0:
#             return
#
#         self._divisions = self._slider.getValue()
#         self.doSplit(joints, self._divisions)
#
#     @staticmethod
#     def doSplit(joints, divisions):
#         for jnt in joints:
#             #get first child, make sure it's a joint
#             jnt2 = jnt.getChildren(type='joint')[0]
#
#             # get distance between joints
#             a = jnt.getTranslation('world')
#             b = jnt2.getTranslation('world')
#             div = (b - a) / divisions
#
#             # increment through divisions and create joints
#             # insert new joints into hierarchy and freeze orientation
#             jnt.select(replace=True)
#             dupe = None
#             for i in xrange(1, divisions):
#                 dupe = pmc.joint(position=a + (div * i))
#                 pmc.makeIdentity(dupe, apply=True, jointOrient=True)
#
#             if dupe:
#                 jnt2.setParent(dupe)

def draw():
    global MAIN_WINDOW
    MAIN_WINDOW = Splitter()
    MAIN_WINDOW.GUI()
