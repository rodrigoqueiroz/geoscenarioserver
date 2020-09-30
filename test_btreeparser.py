#!/usr/bin/env python
#dinizr@chalmers.se

from antlr4 import *
from sv.btree.parser.BTreeDSLLexer import *
from sv.btree.parser.BTreeDSLParser import *
from sv.btree.parser.BTreeDSLListener import *
from sv.btree.BTreeParser import *

def main():
    
    path= "scenarios/trees/"
    scenarios = ["drive_tree.btree","drive_scenario_tree.btree", "fast_drive_scenario_tree.btree", "impatient_drive_scenario_tree.btree", "lane_change_scenario_tree.btree"]
    
    for scenario in scenarios:
        try:
            in_file = open(path+scenario, 'r')
            src = in_file.read()
            lexer = BTreeDSLLexer(InputStream(src))
            parser = BTreeDSLParser(CommonTokenStream(lexer))
            tree = parser.behaviorTree()
            listener = BTreeParser(vid=1,bmodel=None)
            ParseTreeWalker().walk(listener, tree)
            tree = listener.getTreeStr()
            print(tree)
        except:
            raise RuntimeError("Failed at "+ scenario)
        finally:
            print("Scenario " + scenario + " passed!")
    return

if __name__ == '__main__':
    main()
