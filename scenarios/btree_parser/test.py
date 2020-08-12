from antlr4 import *
from BTreeDSLLexer import BTreeDSLLexer
from BTreeDSLParser import BTreeDSLParser
from BTreeDSLListener import BTreeDSLListener

import random
import string

import re

def genstr(i):
    s = i.lower()
    s=re.sub(r'\W+', '', s)
    aux = ""
    for i in range(0,len(s),int(len(s)/3)):
        aux += s[i]
    s = aux[::-1]
    return s

class BuildTree(BTreeDSLListener):

    def __init__(self):
        self.exec_stack=[]
    
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
                        itemm=itemm.replace("[parent]", aux2[1])
                        itemm = itemm.replace("[pos]", aux2[2])
                        aux.append(itemm)
        
        for del_item in del_list:
            self.exec_stack.remove(del_item)
        
        for aux_item in aux:
            self.exec_stack.append(aux_item)

    def exitBehaviorTree(self, ctx):
        s = "tree = trees.BehaviourTree(root=root)"
        self.exec_stack.append(s)
        #print(s)
        s = "tree.setup(timeout=15)"
        self.exec_stack.append(s)
        #print(s)

        self.postProcessExecStack()
        for item in self.exec_stack:
            print(item)

    def exitRootNode(self,ctx):
        node = ctx.node()
        if node.leafNode() != None:
            raise RuntimeError("Root nodes must be operators (e.g. '?', '->', '||').")
        elif node.nodeComposition() != None:
            var = self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + genstr(node.nodeComposition().getText())
        s = "root = " + var
        #print(s)

    def exitNodeComposition(self, ctx): 
        nm = genstr(ctx.getText())
        op = self.map(ctx.OPERATOR().getText())
        var = op.lower()[:3] + "_" + nm
        
        s =  var + " = composites." + op + "(\"" + nm + "\")"
        self.exec_stack.append(s)
        #print(s)
        
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
                    aux.append(self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + genstr(node.nodeComposition().getText()))
                pos += 1
        else:    
            x = "["+genstr(ctx.node().getText()) + "]"

        s = var + ".add_children(" + str(aux).replace("\'","") + ")"
        self.exec_stack.append(s)
        #print(s)

    def exitManeuver(self, ctx): 
        params = ""
        if hasattr(ctx.params(), '__iter__'):
            for param in ctx.params(): params += param.getText()
        else:    
            params = ctx.params().getText()
        s = ctx.name().getText() + " = " + "ManeuverAction(" + ctx.key().getText() +", params={" + params + "})"
        self.exec_stack.append(s)
        #print(s)

    def exitCondition(self, ctx): 
        params = ""
        if hasattr(ctx.params(), '__iter__'):
            for param in ctx.params(): params += param.getText()
        else:    
            params = ctx.params().getText()
        
        s = ctx.name().getText() + " = " + "BCondition(params={" + params + "})"
        self.exec_stack.append(s)
        #print(s)

    #needs to be fixed, how do we reconfigure the subtree?
    # root.id is hardcoded and incorrect
    # should not be added to the pool of children to be added
    # last parameter of the insert_subtree means the position to be inserted
    def exitSubtree(self, ctx): 
        params = ""
        if hasattr(ctx.params(), '__iter__'):
            for param in ctx.params(): params += param.getText()
        else:    
            params = ctx.params().getText()
        s = "tree.insert_subtree(self."+ctx.name().getText()+".get_subtree(), [parent].id, [pos])"
        self.exec_stack.append(s)
        #print(s)

        

def main():
    
    in_file = open('test1.txt', 'r')
    src = in_file.read()
    lexer = BTreeDSLLexer(InputStream(src))
    parser = BTreeDSLParser(CommonTokenStream(lexer))

    tree = parser.behaviorTree()
    #print(tree.getText())
    #print ("Tree " + tree.toStringTree(recog=parser))

    #tree = CommonTree(parser.parse().getTree())
    walker = ParseTreeWalker()
    walker.walk(BuildTree(), tree)
    print()

    return

if __name__ == '__main__':
    main()
