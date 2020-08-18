# Generated from BTreeDSL.g4 by ANTLR 4.7.2
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys


from antlr_denter.DenterHelper import DenterHelper
from sv.btree.parser.BTreeDSLParser import BTreeDSLParser


def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2\22")
        buf.write("\u008f\b\1\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7")
        buf.write("\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\4\r\t\r")
        buf.write("\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\3\2\3\2\3\2\3")
        buf.write("\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\3\3\3\3\4\3\4")
        buf.write("\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\5\3\5\3\6\3\6\3\6\3")
        buf.write("\6\3\6\3\6\3\6\3\6\3\6\3\7\3\7\3\7\3\7\3\7\3\7\3\7\3\7")
        buf.write("\3\b\3\b\3\t\3\t\3\n\3\n\3\n\3\n\3\13\3\13\3\13\3\13\3")
        buf.write("\13\5\13]\n\13\3\f\3\f\3\f\3\f\3\f\3\f\3\f\3\f\3\f\5\f")
        buf.write("h\n\f\3\r\3\r\3\16\5\16m\n\16\3\16\7\16p\n\16\f\16\16")
        buf.write("\16s\13\16\3\16\5\16v\n\16\3\16\6\16y\n\16\r\16\16\16")
        buf.write("z\3\17\6\17~\n\17\r\17\16\17\177\3\20\3\20\3\20\3\20\3")
        buf.write("\21\5\21\u0087\n\21\3\21\3\21\7\21\u008b\n\21\f\21\16")
        buf.write("\21\u008e\13\21\2\2\22\3\3\5\4\7\5\t\6\13\7\r\b\17\t\21")
        buf.write("\n\23\13\25\f\27\r\31\16\33\17\35\20\37\21!\22\3\2\b\4")
        buf.write("\2>>@@\4\2--//\3\2\62;\3\2\60\60\5\2C\\aac|\4\2\13\13")
        buf.write("\"\"\2\u009b\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3\2\2\2\2\t\3")
        buf.write("\2\2\2\2\13\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2\2\2\21\3\2")
        buf.write("\2\2\2\23\3\2\2\2\2\25\3\2\2\2\2\27\3\2\2\2\2\31\3\2\2")
        buf.write("\2\2\33\3\2\2\2\2\35\3\2\2\2\2\37\3\2\2\2\2!\3\2\2\2\3")
        buf.write("#\3\2\2\2\5\60\3\2\2\2\7\62\3\2\2\2\t<\3\2\2\2\13>\3\2")
        buf.write("\2\2\rG\3\2\2\2\17O\3\2\2\2\21Q\3\2\2\2\23S\3\2\2\2\25")
        buf.write("\\\3\2\2\2\27g\3\2\2\2\31i\3\2\2\2\33l\3\2\2\2\35}\3\2")
        buf.write("\2\2\37\u0081\3\2\2\2!\u0086\3\2\2\2#$\7d\2\2$%\7g\2\2")
        buf.write("%&\7j\2\2&\'\7c\2\2\'(\7x\2\2()\7k\2\2)*\7q\2\2*+\7t\2")
        buf.write("\2+,\7v\2\2,-\7t\2\2-.\7g\2\2./\7g\2\2/\4\3\2\2\2\60\61")
        buf.write("\7<\2\2\61\6\3\2\2\2\62\63\7e\2\2\63\64\7q\2\2\64\65\7")
        buf.write("p\2\2\65\66\7f\2\2\66\67\7k\2\2\678\7v\2\289\7k\2\29:")
        buf.write("\7q\2\2:;\7p\2\2;\b\3\2\2\2<=\7.\2\2=\n\3\2\2\2>?\7o\2")
        buf.write("\2?@\7c\2\2@A\7p\2\2AB\7g\2\2BC\7w\2\2CD\7x\2\2DE\7g\2")
        buf.write("\2EF\7t\2\2F\f\3\2\2\2GH\7u\2\2HI\7w\2\2IJ\7d\2\2JK\7")
        buf.write("v\2\2KL\7t\2\2LM\7g\2\2MN\7g\2\2N\16\3\2\2\2OP\7*\2\2")
        buf.write("P\20\3\2\2\2QR\7+\2\2R\22\3\2\2\2ST\7m\2\2TU\7g\2\2UV")
        buf.write("\7{\2\2V\24\3\2\2\2W]\7A\2\2XY\7/\2\2Y]\7@\2\2Z[\7~\2")
        buf.write("\2[]\7~\2\2\\W\3\2\2\2\\X\3\2\2\2\\Z\3\2\2\2]\26\3\2\2")
        buf.write("\2^h\t\2\2\2_`\7?\2\2`h\7?\2\2ab\7@\2\2bh\7?\2\2cd\7>")
        buf.write("\2\2dh\7?\2\2ef\7#\2\2fh\7?\2\2g^\3\2\2\2g_\3\2\2\2ga")
        buf.write("\3\2\2\2gc\3\2\2\2ge\3\2\2\2h\30\3\2\2\2ij\7?\2\2j\32")
        buf.write("\3\2\2\2km\t\3\2\2lk\3\2\2\2lm\3\2\2\2mu\3\2\2\2np\t\4")
        buf.write("\2\2on\3\2\2\2ps\3\2\2\2qo\3\2\2\2qr\3\2\2\2rt\3\2\2\2")
        buf.write("sq\3\2\2\2tv\t\5\2\2uq\3\2\2\2uv\3\2\2\2vx\3\2\2\2wy\t")
        buf.write("\4\2\2xw\3\2\2\2yz\3\2\2\2zx\3\2\2\2z{\3\2\2\2{\34\3\2")
        buf.write("\2\2|~\t\6\2\2}|\3\2\2\2~\177\3\2\2\2\177}\3\2\2\2\177")
        buf.write("\u0080\3\2\2\2\u0080\36\3\2\2\2\u0081\u0082\t\7\2\2\u0082")
        buf.write("\u0083\3\2\2\2\u0083\u0084\b\20\2\2\u0084 \3\2\2\2\u0085")
        buf.write("\u0087\7\17\2\2\u0086\u0085\3\2\2\2\u0086\u0087\3\2\2")
        buf.write("\2\u0087\u0088\3\2\2\2\u0088\u008c\7\f\2\2\u0089\u008b")
        buf.write("\7\"\2\2\u008a\u0089\3\2\2\2\u008b\u008e\3\2\2\2\u008c")
        buf.write("\u008a\3\2\2\2\u008c\u008d\3\2\2\2\u008d\"\3\2\2\2\u008e")
        buf.write("\u008c\3\2\2\2\r\2\\glquz}\177\u0086\u008c\3\b\2\2")
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
    T__8 = 9
    OPERATOR = 10
    BOP = 11
    ATT = 12
    FLOAT = 13
    WORD = 14
    WS = 15
    NL = 16

    channelNames = [ u"DEFAULT_TOKEN_CHANNEL", u"HIDDEN" ]

    modeNames = [ "DEFAULT_MODE" ]

    literalNames = [ "<INVALID>",
            "'behaviortree'", "':'", "'condition'", "','", "'maneuver'", 
            "'subtree'", "'('", "')'", "'key'", "'='" ]

    symbolicNames = [ "<INVALID>",
            "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", "WS", "NL" ]

    ruleNames = [ "T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6", 
                  "T__7", "T__8", "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", 
                  "WS", "NL" ]

    grammarFileName = "BTreeDSL.g4"

    def __init__(self, input=None, output:TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.7.2")
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



