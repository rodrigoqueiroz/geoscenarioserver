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
        self.stack=[]
    
    def map(self, symbol):
        if symbol == "->" : return "Sequence" 
        if symbol == "?"  : return "Selector"
        if symbol == "||" : return "Parallel"
        
        raise RuntimeError("No symbol matched (" + symbol + ")")

    def exitNodeComposition(self, ctx): 
        nm = genstr(ctx.getText())
        op = self.map(ctx.OPERATOR().getText())
        var = op.lower()[:3] + "_" + nm
        
        s =  var + " = composites." + op + "(\"" + nm + "\")"
        self.stack.append(s)
        print(s)
        
        x = []

        if hasattr(ctx.node(), '__iter__'):
            for node in ctx.node():
                if node.leafNode() == None:
                    x.append(self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + genstr(node.nodeComposition().getText()))

                elif node.nodeComposition() == None:
                    if node.leafNode().maneuver() != None:
                        x.append(node.leafNode().maneuver().name().getText())
                    elif node.leafNode().condition() != None:
                        x.append(node.leafNode().condition().name().getText())
                    elif node.leafNode().subtree() != None:
                        x.append(node.leafNode().subtree().name().getText())

        else:    
            x = "["+genstr(ctx.node().getText()) + "]"

        s = var + ".add_children(" + str(x) + ")"
        self.stack.append(s)
        print(s)

    def exitManeuver(self, ctx): 
        params = ""
        if hasattr(ctx.params(), '__iter__'):
            for param in ctx.params(): params += param.getText()
        else:    
            params = ctx.params().getText()
        s = ctx.name().getText() + " = " + "ManeuverAction(" + ctx.key().getText() +", params={" + params + "})"
        self.stack.append(s)
        print(s)

    def exitCondition(self, ctx): 
        params = ""
        if hasattr(ctx.params(), '__iter__'):
            for param in ctx.params(): params += param.getText()
        else:    
            params = ctx.params().getText()
        
        s = ctx.name().getText() + " = " + "BCondition(params={" + params + "})"
        self.stack.append(s)
        print(s)

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
        s = "tree.insert_subtree(self."+ctx.name().getText()+".get_subtree(), root.id, 1)"
        self.stack.append(s)
        print(s)

        

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
