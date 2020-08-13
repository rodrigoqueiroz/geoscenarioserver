#!/usr/bin/env python
#dinizr@chalmers.se

from antlr4 import *
from sv.btree.parser.BTreeDSLLexer import *
from sv.btree.parser.BTreeDSLParser import *
from sv.btree.parser.BTreeDSLListener import *
from sv.btree.BTreeParser import *

def main():
    
    in_file = open('test1.txt', 'r')
    src = in_file.read()
    lexer = BTreeDSLLexer(InputStream(src))
    parser = BTreeDSLParser(CommonTokenStream(lexer))

    tree = parser.behaviorTree()
    walker = ParseTreeWalker()
    walker.walk(BTreeParser.BTreeListener(1), tree)
    print()

    return

if __name__ == '__main__':
    main()
