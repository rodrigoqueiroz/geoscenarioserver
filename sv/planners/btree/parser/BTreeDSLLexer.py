# Generated from BTreeDSL.g4 by ANTLR 4.7.2
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys


from antlr_denter.DenterHelper import DenterHelper
from sv.planners.btree.parser.BTreeDSLParser import BTreeDSLParser


def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2\21")
        buf.write("\u0089\b\1\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7")
        buf.write("\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\4\r\t\r")
        buf.write("\4\16\t\16\4\17\t\17\4\20\t\20\3\2\3\2\3\2\3\2\3\2\3\2")
        buf.write("\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\3\3\3\3\4\3\4\3\4\3\4\3")
        buf.write("\4\3\4\3\4\3\4\3\4\3\4\3\5\3\5\3\6\3\6\3\7\3\7\3\7\3\7")
        buf.write("\3\7\3\7\3\7\3\7\3\7\3\b\3\b\3\b\3\b\3\b\3\b\3\b\3\b\3")
        buf.write("\t\3\t\3\n\3\n\3\n\3\n\3\n\5\nW\n\n\3\13\3\13\3\13\3\13")
        buf.write("\3\13\3\13\3\13\3\13\3\13\5\13b\n\13\3\f\3\f\3\r\5\rg")
        buf.write("\n\r\3\r\7\rj\n\r\f\r\16\rm\13\r\3\r\5\rp\n\r\3\r\6\r")
        buf.write("s\n\r\r\r\16\rt\3\16\6\16x\n\16\r\16\16\16y\3\17\3\17")
        buf.write("\3\17\3\17\3\20\5\20\u0081\n\20\3\20\3\20\7\20\u0085\n")
        buf.write("\20\f\20\16\20\u0088\13\20\2\2\21\3\3\5\4\7\5\t\6\13\7")
        buf.write("\r\b\17\t\21\n\23\13\25\f\27\r\31\16\33\17\35\20\37\21")
        buf.write("\3\2\b\4\2>>@@\4\2--//\3\2\62;\3\2\60\60\5\2C\\aac|\4")
        buf.write("\2\13\13\"\"\2\u0095\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3\2\2")
        buf.write("\2\2\t\3\2\2\2\2\13\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2\2\2")
        buf.write("\21\3\2\2\2\2\23\3\2\2\2\2\25\3\2\2\2\2\27\3\2\2\2\2\31")
        buf.write("\3\2\2\2\2\33\3\2\2\2\2\35\3\2\2\2\2\37\3\2\2\2\3!\3\2")
        buf.write("\2\2\5.\3\2\2\2\7\60\3\2\2\2\t:\3\2\2\2\13<\3\2\2\2\r")
        buf.write(">\3\2\2\2\17G\3\2\2\2\21O\3\2\2\2\23V\3\2\2\2\25a\3\2")
        buf.write("\2\2\27c\3\2\2\2\31f\3\2\2\2\33w\3\2\2\2\35{\3\2\2\2\37")
        buf.write("\u0080\3\2\2\2!\"\7d\2\2\"#\7g\2\2#$\7j\2\2$%\7c\2\2%")
        buf.write("&\7x\2\2&\'\7k\2\2\'(\7q\2\2()\7t\2\2)*\7v\2\2*+\7t\2")
        buf.write("\2+,\7g\2\2,-\7g\2\2-\4\3\2\2\2./\7<\2\2/\6\3\2\2\2\60")
        buf.write("\61\7e\2\2\61\62\7q\2\2\62\63\7p\2\2\63\64\7f\2\2\64\65")
        buf.write("\7k\2\2\65\66\7v\2\2\66\67\7k\2\2\678\7q\2\289\7p\2\2")
        buf.write("9\b\3\2\2\2:;\7*\2\2;\n\3\2\2\2<=\7+\2\2=\f\3\2\2\2>?")
        buf.write("\7o\2\2?@\7c\2\2@A\7p\2\2AB\7g\2\2BC\7w\2\2CD\7x\2\2D")
        buf.write("E\7g\2\2EF\7t\2\2F\16\3\2\2\2GH\7u\2\2HI\7w\2\2IJ\7d\2")
        buf.write("\2JK\7v\2\2KL\7t\2\2LM\7g\2\2MN\7g\2\2N\20\3\2\2\2OP\7")
        buf.write(".\2\2P\22\3\2\2\2QW\7A\2\2RS\7/\2\2SW\7@\2\2TU\7~\2\2")
        buf.write("UW\7~\2\2VQ\3\2\2\2VR\3\2\2\2VT\3\2\2\2W\24\3\2\2\2Xb")
        buf.write("\t\2\2\2YZ\7?\2\2Zb\7?\2\2[\\\7@\2\2\\b\7?\2\2]^\7>\2")
        buf.write("\2^b\7?\2\2_`\7#\2\2`b\7?\2\2aX\3\2\2\2aY\3\2\2\2a[\3")
        buf.write("\2\2\2a]\3\2\2\2a_\3\2\2\2b\26\3\2\2\2cd\7?\2\2d\30\3")
        buf.write("\2\2\2eg\t\3\2\2fe\3\2\2\2fg\3\2\2\2go\3\2\2\2hj\t\4\2")
        buf.write("\2ih\3\2\2\2jm\3\2\2\2ki\3\2\2\2kl\3\2\2\2ln\3\2\2\2m")
        buf.write("k\3\2\2\2np\t\5\2\2ok\3\2\2\2op\3\2\2\2pr\3\2\2\2qs\t")
        buf.write("\4\2\2rq\3\2\2\2st\3\2\2\2tr\3\2\2\2tu\3\2\2\2u\32\3\2")
        buf.write("\2\2vx\t\6\2\2wv\3\2\2\2xy\3\2\2\2yw\3\2\2\2yz\3\2\2\2")
        buf.write("z\34\3\2\2\2{|\t\7\2\2|}\3\2\2\2}~\b\17\2\2~\36\3\2\2")
        buf.write("\2\177\u0081\7\17\2\2\u0080\177\3\2\2\2\u0080\u0081\3")
        buf.write("\2\2\2\u0081\u0082\3\2\2\2\u0082\u0086\7\f\2\2\u0083\u0085")
        buf.write("\7\"\2\2\u0084\u0083\3\2\2\2\u0085\u0088\3\2\2\2\u0086")
        buf.write("\u0084\3\2\2\2\u0086\u0087\3\2\2\2\u0087 \3\2\2\2\u0088")
        buf.write("\u0086\3\2\2\2\r\2Vafkotwy\u0080\u0086\3\b\2\2")
        return buf.getvalue()


class BTreeDSLLexer(Lexer):

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [ DFA(ds, i) for i, ds in enumerate(atn.decisionToState) ]

    T__0 = 1
    T__1 = 2
    T__2 = 3
    T__3 = 4
    T__4 = 5
    T__5 = 6
    T__6 = 7
    T__7 = 8
    OPERATOR = 9
    BOP = 10
    ATT = 11
    FLOAT = 12
    WORD = 13
    WS = 14
    NL = 15

    channelNames = [ u"DEFAULT_TOKEN_CHANNEL", u"HIDDEN" ]

    modeNames = [ "DEFAULT_MODE" ]

    literalNames = [ "<INVALID>",
            "'behaviortree'", "':'", "'condition'", "'('", "')'", "'maneuver'", 
            "'subtree'", "','", "'='" ]

    symbolicNames = [ "<INVALID>",
            "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", "WS", "NL" ]

    ruleNames = [ "T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6", 
                  "T__7", "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", "WS", 
                  "NL" ]

    grammarFileName = "BTreeDSL.g4"

    def __init__(self, input=None, output:TextIO = sys.stdout):
        super().__init__(input, output)
        #self.checkVersion("4.7.2")
        self._interp = LexerATNSimulator(self, self.atn, self.decisionsToDFA, PredictionContextCache())
        self._actions = None
        self._predicates = None


    class BTreeDSLDenter(DenterHelper):
        def __init__(self, lexer, nl_token, indent_token, dedent_token, ignore_eof):
            super().__init__(nl_token, indent_token, dedent_token, ignore_eof)
            self.lexer: BTreeDSLLexer = lexer

        def pull_token(self):
            return super(BTreeDSLLexer, self.lexer).nextToken()

    denter = None

    def nextToken(self):
        if not self.denter:
            self.denter = self.BTreeDSLDenter(self, self.NL, BTreeDSLParser.INDENT, BTreeDSLParser.DEDENT, ignore_eof=False)
        return self.denter.next_token()



