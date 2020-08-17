#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#dinizr@chalmers.se

from py_trees import *
from sv.ManeuverConfig import MVelKeepConfig
from sv.ManeuverUtils import *
from sv.btree.BTreeLeaves import *

from antlr4 import *
from sv.btree.parser.BTreeDSLLexer import BTreeDSLLexer
from sv.btree.parser.BTreeDSLParser import BTreeDSLParser
from sv.btree.parser.BTreeDSLListener import BTreeDSLListener

import random
import string

import re
import os

'''For readability:
    st_ = subtree
    seq_    = sequence  (progressively tick each node)
    sel_    = selector  (decision makers, ticks each node until one succeed)
    par_    = parallel
    m_      = maneuver (action)
    c_      = condition
    - Cross cutting maneuver configuration (that will apply to the whole tree)
    must be passed from the root tree (scenario tree)
    - Alternative configurations from the same maneuver
    must be handled with a specialized tree (create and modify)
'''

class BTreeParser(object):
    def __init__(self, vid):
        self.vid = vid

    def parse_tree(self, bmodel, btree_name, textual_model):
        lexer = BTreeDSLLexer(InputStream(textual_model))
        parser = BTreeDSLParser(CommonTokenStream(lexer))

        ast = parser.behaviorTree()
        walker = ParseTreeWalker()
        walker.walk(self.BTreeListener(self.vid), ast)

        # instantiate the parsed tree
        path = "./sv/btree/"
        ff = open(path + "tmp" + str(self.vid) + ".btree", 'r')
        behavior_tree = ff.read()
        # print(behavior_tree)
        try:
            exec(behavior_tree, globals()) #defines get_tree
            tree = get_tree(self, bmodel)
        except Exception:
            raise RuntimeError("Could not set behavior tree up.")
        ff.close()
        os.remove(path + "tmp" + str(self.vid) + ".btree")
        #return getattr(self,root_btree_name)(bmodel)
        return tree

    #REUSABLE TREES
    def drive_tree(self, bmodel, mvk_config=MVelKeepConfig(), mstop_config=MStopConfig()):
        #Drive SubTree
        #maneuvers
        m_vk = ManeuverAction(bmodel,"m vel keep", mvk_config)
        m_follow = ManeuverAction(bmodel,"m follow", MFollowConfig())
        m_stop = ManeuverAction(bmodel,"m stop", mstop_config)
        #conditions
        c_lane_occupied = BCondition(bmodel,"c lane_occupied?", "lane_occupied")
        c_lv_stop = BCondition(bmodel,"c lv_stopped?", "lv_stopped")
        c_reached_goal = BCondition(bmodel, "c reached_goal", "reached_goal")

        #Coordinate Maneuvers and Conditions
        # stopping when leading vehicle is stopped shouldn't have to be a separate behaviour,
        # follow maneuver should just do that. Also this will never be triggered because the
        # condition is broken (not keeping leading_vehicle in behaviour models anymore)
        # seq_lv_stop = composites.Sequence("seq lv stop")
        # seq_lv_stop.add_children([c_lv_stop, ManeuverAction(bmodel, "stop when lv stopped", mstop_config)])
        # if reached goal, do stop maneuver
        seq_reached_goal = composites.Sequence("seq reached goal")
        seq_reached_goal.add_children([c_reached_goal, m_stop])

        # sel_lv = composites.Selector("sel lv")
        # sel_lv.add_children([seq_lv_stop, m_follow])

        seq_busy_lane = composites.Sequence("seq busy Lane")
        seq_busy_lane.add_children([c_lane_occupied, m_follow])

        st_sel_drive = composites.Selector("ST Drive")
        st_sel_drive.add_children([seq_reached_goal, seq_busy_lane, m_vk])

        return st_sel_drive

    def lane_change_tree(self, bmodel, target=1):
        m_lane_swerve = ManeuverAction(bmodel, "m lane swerve", MLaneSwerveConfig(target_lid=target))
        m_cutin = ManeuverAction(bmodel, "m cutin", MCutInConfig(target_lid=target))
        c_should_cutin = BCondition(bmodel, "c should do cutin?", "should_cutin", target_lane_id=target)

        # determine whether to do cutin
        seq_cutin = composites.Sequence("seq cutin", children=[c_should_cutin, m_cutin])

        sel_cutin_or_lane_swerve = composites.Selector("sel cutin or lane swerve", children=[seq_cutin, m_lane_swerve])

        return sel_cutin_or_lane_swerve

    def reverse_tree(self, bmodel):
        m_reverse = ManeuverAction(bmodel, "m reverse", MReverseConfig())
        m_stop = ManeuverAction(bmodel,"m stop", MStopConfig())
        c_reached_goal = BCondition(bmodel, "c reached_goal", "reached_goal", reverse=True)

        # if reached goal, do stop maneuver
        seq_reached_goal = composites.Sequence("seq reached goal")
        seq_reached_goal.add_children([c_reached_goal, m_stop])

        st_sel_reverse = composites.Selector("ST Reverse")
        st_sel_reverse.add_children([seq_reached_goal, m_reverse])

        return st_sel_reverse

    class BTreeListener(BTreeDSLListener):

        def __init__(self, vid):
            self.exec_stack = []
            self.vid = vid

        def genstr(self, i):
            # gen 10 word random string
            #j = ''.join(random.choice(string.ascii_letters) for x in range(10))
            # interleave input with randword
            #res = "".join(m + n for m, n in zip(i, j))
            # to_lower
            s = i.lower()
            # remove non-alphabetical chars
            s=re.sub(r'\W+', '', s)
            #reduce the string size, 3-fold
            aux = ""
            for i in range(0,len(s),int(len(s)/3)):
                aux += s[i]
            s = aux[::-1]
            return s

        def map(self, symbol):
            if symbol == "->" : return "Sequence"
            if symbol == "?"  : return "Selector"
            if symbol == "||" : return "Parallel"

            raise RuntimeError("No symbol matched (" + symbol + ")")

        def postProcessExecStack(self):
            self.sortExecStack()
            self.completeInsertSubtree()

        def sortExecStack(self):
            aux = []
            for item in self.exec_stack:
                if re.search("tree.insert_subtree",item):
                    aux.append(item)

            for aux_item in aux:
                self.exec_stack.remove(aux_item)
                self.exec_stack.append(aux_item)

        def completeInsertSubtree(self):
            aux = []
            del_list = []
            for item in self.exec_stack:
                if re.search("~", item):
                    aux2 = item[1:].split(",")
                    self.exec_stack.remove(item)
                    for itemm in self.exec_stack:
                        if re.search(aux2[0], itemm):
                            del_list.append(itemm)
                            itemm = itemm.replace("[parent]", aux2[1])
                            itemm = itemm.replace("[pos]", aux2[2])
                            aux.append(itemm)

            for del_item in del_list:
                self.exec_stack.remove(del_item)

            for aux_item in aux:
                self.exec_stack.append(aux_item)

        def build_tree(self):
            path="./sv/btree/"
            of = open(path+'tmp'+str(self.vid)+".btree",'w')
            of.write("def get_tree(parser,bmodel):\n")
            for item in self.exec_stack:
                of.write("    "+item+"\n")
            of.write("    return tree")
            of.close()

        def exitBehaviorTree(self, ctx):
            s = "tree = trees.BehaviourTree(root=root)"
            self.exec_stack.append(s)

            self.postProcessExecStack()
            self.build_tree()

        def exitRootNode(self,ctx):
            node = ctx.node()
            if node.leafNode() != None:
                raise RuntimeError("The root node must be an operator (e.g. '?', '->', '||').")
            elif node.nodeComposition() != None:
                var = self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + self.genstr(node.nodeComposition().getText())
            s = "root = " + var
            self.exec_stack.append(s)

        def exitNodeComposition(self, ctx):
            nm = self.genstr(ctx.getText())
            op = self.map(ctx.OPERATOR().getText())
            var = op.lower()[:3] + "_" + nm

            s =  var + " = composites." + op + "(\"" + var + "\")"
            self.exec_stack.append(s)

            aux = []
            if hasattr(ctx.node(), '__iter__'):
                pos=0
                for node in ctx.node():

                    if node.leafNode() != None:
                        if node.leafNode().maneuver() != None:
                            aux.append(node.leafNode().maneuver().name().getText())
                        elif node.leafNode().condition() != None:
                            aux.append(node.leafNode().condition().name().getText())
                        elif node.leafNode().subtree() != None:     self.exec_stack.append("~" + node.leafNode().subtree().name().getText()+","+var+","+str(pos))

                    elif node.nodeComposition() != None:
                        aux.append(self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + self.genstr(node.nodeComposition().getText()))
                    pos += 1
            else:
                aux = "["+self.genstr(ctx.node().getText()) + "]"

            s = var + ".add_children(" + str(aux).replace("\'","") + ")"
            self.exec_stack.append(s)

        # e.g. stop = ManeuverAction(bmodel, "stop", MStopConfig(time=2, dist=10, decel=3))
        def exitManeuver(self, ctx):
            name=ctx.name().getText()
            mconfig = ctx.mconfig().getText()
            s = name + " = " + "ManeuverAction(bmodel," + "\""+ name + "\""+"," + mconfig + ")"
            self.exec_stack.append(s)

        # e.g. endpoint = BCondition(bmodel,"endpoint", "lane_occupied", args)
        def exitCondition(self, ctx):
            name=ctx.name().getText()
            params = ""
            if hasattr(ctx.params(), '__iter__'):
                for param in ctx.params(): params += param.getText()
            else:
                params = ctx.params().getText()
            key = ctx.key().name().getText()
            s = ctx.name().getText() + " = " + "BCondition(bmodel," + "\"" + name + "\"" + "," + "\"" + key + "\""
            if params != "": s = s + "," + params
            s += ")"
            self.exec_stack.append(s)

        def exitSubtree(self, ctx):
            mconfigs = ""
            if hasattr(ctx.mconfig(), '__iter__'):
                for conf in ctx.mconfig(): mconfigs += "," + conf.getText()
            else:
                mconfigs = ctx.mconfig().getText()
            s = "tree.insert_subtree(parser."+ctx.name().getText()+"(bmodel"
            if mconfigs != "": s += mconfigs
            s += "), [parent].id, [pos])"
            self.exec_stack.append(s)
