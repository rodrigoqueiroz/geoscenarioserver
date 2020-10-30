# Generated from BTreeDSL.g4 by ANTLR 4.7.2
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys


from antlr_denter.DenterHelper import DenterHelper
from sv.btree.parser.BTreeDSLParser import BTreeDSLParser

def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2\23")
        buf.write("\u0099\b\1\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7")
        buf.write("\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\4\r\t\r")
        buf.write("\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22\t\22\3\2")
        buf.write("\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\3\3")
        buf.write("\3\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\4\3\5\3\5\3\6")
        buf.write("\3\6\3\7\3\7\3\b\3\b\3\b\3\b\3\b\3\b\3\b\3\b\3\b\3\t\3")
        buf.write("\t\3\t\3\t\3\t\3\t\3\t\3\t\3\n\3\n\3\n\3\n\3\n\3\n\3\13")
        buf.write("\3\13\3\13\3\13\3\13\3\13\3\f\3\f\3\f\3\f\3\f\5\fg\n\f")
        buf.write("\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\5\rr\n\r\3\16\3\16")
        buf.write("\3\17\5\17w\n\17\3\17\7\17z\n\17\f\17\16\17}\13\17\3\17")
        buf.write("\5\17\u0080\n\17\3\17\6\17\u0083\n\17\r\17\16\17\u0084")
        buf.write("\3\20\6\20\u0088\n\20\r\20\16\20\u0089\3\21\3\21\3\21")
        buf.write("\3\21\3\22\5\22\u0091\n\22\3\22\3\22\7\22\u0095\n\22\f")
        buf.write("\22\16\22\u0098\13\22\2\2\23\3\3\5\4\7\5\t\6\13\7\r\b")
        buf.write("\17\t\21\n\23\13\25\f\27\r\31\16\33\17\35\20\37\21!\22")
        buf.write("#\23\3\2\b\4\2>>@@\4\2--//\3\2\62;\3\2\60\60\5\2C\\aa")
        buf.write("c|\4\2\13\13\"\"\2\u00a5\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3")
        buf.write("\2\2\2\2\t\3\2\2\2\2\13\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2")
        buf.write("\2\2\21\3\2\2\2\2\23\3\2\2\2\2\25\3\2\2\2\2\27\3\2\2\2")
        buf.write("\2\31\3\2\2\2\2\33\3\2\2\2\2\35\3\2\2\2\2\37\3\2\2\2\2")
        buf.write("!\3\2\2\2\2#\3\2\2\2\3%\3\2\2\2\5\62\3\2\2\2\7\64\3\2")
        buf.write("\2\2\t>\3\2\2\2\13@\3\2\2\2\rB\3\2\2\2\17D\3\2\2\2\21")
        buf.write("M\3\2\2\2\23U\3\2\2\2\25[\3\2\2\2\27f\3\2\2\2\31q\3\2")
        buf.write("\2\2\33s\3\2\2\2\35v\3\2\2\2\37\u0087\3\2\2\2!\u008b\3")
        buf.write("\2\2\2#\u0090\3\2\2\2%&\7d\2\2&\'\7g\2\2\'(\7j\2\2()\7")
        buf.write("c\2\2)*\7x\2\2*+\7k\2\2+,\7q\2\2,-\7t\2\2-.\7v\2\2./\7")
        buf.write("t\2\2/\60\7g\2\2\60\61\7g\2\2\61\4\3\2\2\2\62\63\7<\2")
        buf.write("\2\63\6\3\2\2\2\64\65\7e\2\2\65\66\7q\2\2\66\67\7p\2\2")
        buf.write("\678\7f\2\289\7k\2\29:\7v\2\2:;\7k\2\2;<\7q\2\2<=\7p\2")
        buf.write("\2=\b\3\2\2\2>?\7*\2\2?\n\3\2\2\2@A\7.\2\2A\f\3\2\2\2")
        buf.write("BC\7+\2\2C\16\3\2\2\2DE\7o\2\2EF\7c\2\2FG\7p\2\2GH\7g")
        buf.write("\2\2HI\7w\2\2IJ\7x\2\2JK\7g\2\2KL\7t\2\2L\20\3\2\2\2M")
        buf.write("N\7u\2\2NO\7w\2\2OP\7d\2\2PQ\7v\2\2QR\7t\2\2RS\7g\2\2")
        buf.write("ST\7g\2\2T\22\3\2\2\2UV\7g\2\2VW\7t\2\2WX\7t\2\2XY\7q")
        buf.write("\2\2YZ\7t\2\2Z\24\3\2\2\2[\\\7f\2\2\\]\7g\2\2]^\7n\2\2")
        buf.write("^_\7c\2\2_`\7{\2\2`\26\3\2\2\2ag\7A\2\2bc\7/\2\2cg\7@")
        buf.write("\2\2de\7~\2\2eg\7~\2\2fa\3\2\2\2fb\3\2\2\2fd\3\2\2\2g")
        buf.write("\30\3\2\2\2hr\t\2\2\2ij\7?\2\2jr\7?\2\2kl\7@\2\2lr\7?")
        buf.write("\2\2mn\7>\2\2nr\7?\2\2op\7#\2\2pr\7?\2\2qh\3\2\2\2qi\3")
        buf.write("\2\2\2qk\3\2\2\2qm\3\2\2\2qo\3\2\2\2r\32\3\2\2\2st\7?")
        buf.write("\2\2t\34\3\2\2\2uw\t\3\2\2vu\3\2\2\2vw\3\2\2\2w\177\3")
        buf.write("\2\2\2xz\t\4\2\2yx\3\2\2\2z}\3\2\2\2{y\3\2\2\2{|\3\2\2")
        buf.write("\2|~\3\2\2\2}{\3\2\2\2~\u0080\t\5\2\2\177{\3\2\2\2\177")
        buf.write("\u0080\3\2\2\2\u0080\u0082\3\2\2\2\u0081\u0083\t\4\2\2")
        buf.write("\u0082\u0081\3\2\2\2\u0083\u0084\3\2\2\2\u0084\u0082\3")
        buf.write("\2\2\2\u0084\u0085\3\2\2\2\u0085\36\3\2\2\2\u0086\u0088")
        buf.write("\t\6\2\2\u0087\u0086\3\2\2\2\u0088\u0089\3\2\2\2\u0089")
        buf.write("\u0087\3\2\2\2\u0089\u008a\3\2\2\2\u008a \3\2\2\2\u008b")
        buf.write("\u008c\t\7\2\2\u008c\u008d\3\2\2\2\u008d\u008e\b\21\2")
        buf.write("\2\u008e\"\3\2\2\2\u008f\u0091\7\17\2\2\u0090\u008f\3")
        buf.write("\2\2\2\u0090\u0091\3\2\2\2\u0091\u0092\3\2\2\2\u0092\u0096")
        buf.write("\7\f\2\2\u0093\u0095\7\"\2\2\u0094\u0093\3\2\2\2\u0095")
        buf.write("\u0098\3\2\2\2\u0096\u0094\3\2\2\2\u0096\u0097\3\2\2\2")
        buf.write("\u0097$\3\2\2\2\u0098\u0096\3\2\2\2\r\2fqv{\177\u0084")
        buf.write("\u0087\u0089\u0090\u0096\3\b\2\2")
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
    T__9 = 10
    OPERATOR = 11
    BOP = 12
    ATT = 13
    FLOAT = 14
    WORD = 15
    WS = 16
    NL = 17

    channelNames = [ u"DEFAULT_TOKEN_CHANNEL", u"HIDDEN" ]

    modeNames = [ "DEFAULT_MODE" ]

    literalNames = [ "<INVALID>",
            "'behaviortree'", "':'", "'condition'", "'('", "','", "')'", 
            "'maneuver'", "'subtree'", "'error'", "'delay'", "'='" ]

    symbolicNames = [ "<INVALID>",
            "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", "WS", "NL" ]

    ruleNames = [ "T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6", 
                  "T__7", "T__8", "T__9", "OPERATOR", "BOP", "ATT", "FLOAT", 
                  "WORD", "WS", "NL" ]

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



