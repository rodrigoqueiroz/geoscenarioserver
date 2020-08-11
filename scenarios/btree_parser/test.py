from antlr4 import *
from BTreeDSLLexer import BTreeDSLLexer
from BTreeDSLParser import BTreeDSLParser

def main():
    
    in_file = open('test1.txt', 'r')
    src = in_file.read()
    print('running lexer...')
    lexer = BTreeDSLLexer(InputStream(src))
    print('running parser...')
    parser = BTreeDSLParser(CommonTokenStream(lexer))
    print('parsed!')

    tree = parser.behavior_tree()
    #print(tree.getText())
    print ("Tree " + tree.toStringTree(recog=parser))

    return

if __name__ == '__main__':
    main()
