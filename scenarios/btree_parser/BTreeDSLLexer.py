# Generated from BTreeDSL.g4 by ANTLR 4.7.2
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys


from antlr_denter.DenterHelper import DenterHelper
from BTreeDSLParser import BTreeDSLParser


def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2\22")
        buf.write("\u008d\b\1\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7")
        buf.write("\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\4\r\t\r")
        buf.write("\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\3\2\3\2\3\2\3")
        buf.write("\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\3\3\3\3\4\3\4")
        buf.write("\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\5\3\5\3\6\3\6\3\7\3")
        buf.write("\7\3\7\3\7\3\7\3\7\3\7\3\7\3\7\3\b\3\b\3\t\3\t\3\t\3\t")
        buf.write("\3\t\3\t\3\t\3\t\3\n\3\n\3\n\3\n\3\13\3\13\3\13\3\13\3")
        buf.write("\13\5\13]\n\13\3\f\3\f\3\f\3\f\3\f\3\f\3\f\3\f\3\f\5\f")
        buf.write("h\n\f\3\r\3\r\3\16\5\16m\n\16\3\16\7\16p\n\16\f\16\16")
        buf.write("\16s\13\16\3\16\5\16v\n\16\3\16\6\16y\n\16\r\16\16\16")
        buf.write("z\3\17\6\17~\n\17\r\17\16\17\177\3\20\3\20\3\21\5\21\u0085")
        buf.write("\n\21\3\21\3\21\7\21\u0089\n\21\f\21\16\21\u008c\13\21")
        buf.write("\2\2\22\3\3\5\4\7\5\t\6\13\7\r\b\17\t\21\n\23\13\25\f")
        buf.write("\27\r\31\16\33\17\35\20\37\21!\22\3\2\b\4\2>>@@\4\2--")
        buf.write("//\3\2\62;\3\2\60\60\5\2C\\aac|\4\2\13\13\"\"\2\u0099")
        buf.write("\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3\2\2\2\2\t\3\2\2\2\2\13")
        buf.write("\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2\2\2\21\3\2\2\2\2\23\3")
        buf.write("\2\2\2\2\25\3\2\2\2\2\27\3\2\2\2\2\31\3\2\2\2\2\33\3\2")
        buf.write("\2\2\2\35\3\2\2\2\2\37\3\2\2\2\2!\3\2\2\2\3#\3\2\2\2\5")
        buf.write("\60\3\2\2\2\7\62\3\2\2\2\t<\3\2\2\2\13>\3\2\2\2\r@\3\2")
        buf.write("\2\2\17I\3\2\2\2\21K\3\2\2\2\23S\3\2\2\2\25\\\3\2\2\2")
        buf.write("\27g\3\2\2\2\31i\3\2\2\2\33l\3\2\2\2\35}\3\2\2\2\37\u0081")
        buf.write("\3\2\2\2!\u0084\3\2\2\2#$\7D\2\2$%\7g\2\2%&\7j\2\2&\'")
        buf.write("\7c\2\2\'(\7x\2\2()\7k\2\2)*\7q\2\2*+\7t\2\2+,\7V\2\2")
        buf.write(",-\7t\2\2-.\7g\2\2./\7g\2\2/\4\3\2\2\2\60\61\7<\2\2\61")
        buf.write("\6\3\2\2\2\62\63\7E\2\2\63\64\7q\2\2\64\65\7p\2\2\65\66")
        buf.write("\7f\2\2\66\67\7k\2\2\678\7v\2\289\7k\2\29:\7q\2\2:;\7")
        buf.write("p\2\2;\b\3\2\2\2<=\7*\2\2=\n\3\2\2\2>?\7+\2\2?\f\3\2\2")
        buf.write("\2@A\7O\2\2AB\7c\2\2BC\7p\2\2CD\7g\2\2DE\7w\2\2EF\7x\2")
        buf.write("\2FG\7g\2\2GH\7t\2\2H\16\3\2\2\2IJ\7.\2\2J\20\3\2\2\2")
        buf.write("KL\7U\2\2LM\7w\2\2MN\7d\2\2NO\7v\2\2OP\7t\2\2PQ\7g\2\2")
        buf.write("QR\7g\2\2R\22\3\2\2\2ST\7m\2\2TU\7g\2\2UV\7{\2\2V\24\3")
        buf.write("\2\2\2W]\7A\2\2XY\7/\2\2Y]\7@\2\2Z[\7~\2\2[]\7~\2\2\\")
        buf.write("W\3\2\2\2\\X\3\2\2\2\\Z\3\2\2\2]\26\3\2\2\2^h\t\2\2\2")
        buf.write("_`\7?\2\2`h\7?\2\2ab\7@\2\2bh\7?\2\2cd\7>\2\2dh\7?\2\2")
        buf.write("ef\7#\2\2fh\7?\2\2g^\3\2\2\2g_\3\2\2\2ga\3\2\2\2gc\3\2")
        buf.write("\2\2ge\3\2\2\2h\30\3\2\2\2ij\7?\2\2j\32\3\2\2\2km\t\3")
        buf.write("\2\2lk\3\2\2\2lm\3\2\2\2mu\3\2\2\2np\t\4\2\2on\3\2\2\2")
        buf.write("ps\3\2\2\2qo\3\2\2\2qr\3\2\2\2rt\3\2\2\2sq\3\2\2\2tv\t")
        buf.write("\5\2\2uq\3\2\2\2uv\3\2\2\2vx\3\2\2\2wy\t\4\2\2xw\3\2\2")
        buf.write("\2yz\3\2\2\2zx\3\2\2\2z{\3\2\2\2{\34\3\2\2\2|~\t\6\2\2")
        buf.write("}|\3\2\2\2~\177\3\2\2\2\177}\3\2\2\2\177\u0080\3\2\2\2")
        buf.write("\u0080\36\3\2\2\2\u0081\u0082\t\7\2\2\u0082 \3\2\2\2\u0083")
        buf.write("\u0085\7\17\2\2\u0084\u0083\3\2\2\2\u0084\u0085\3\2\2")
        buf.write("\2\u0085\u0086\3\2\2\2\u0086\u008a\7\f\2\2\u0087\u0089")
        buf.write("\7\"\2\2\u0088\u0087\3\2\2\2\u0089\u008c\3\2\2\2\u008a")
        buf.write("\u0088\3\2\2\2\u008a\u008b\3\2\2\2\u008b\"\3\2\2\2\u008c")
        buf.write("\u008a\3\2\2\2\r\2\\glquz}\177\u0084\u008a\2")
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
            "'BehaviorTree'", "':'", "'Condition'", "'('", "')'", "'Maneuver'", 
            "','", "'Subtree'", "'key'", "'='" ]

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


