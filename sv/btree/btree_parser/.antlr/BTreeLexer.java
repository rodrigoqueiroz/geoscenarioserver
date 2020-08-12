// Generated from /home/rcaldas/phd/research/explore_research_topic/behavior_trees/impl/geoscenarioserver/scenarios/btree_parser/BTree.g4 by ANTLR 4.8
import org.antlr.v4.runtime.Lexer;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.TokenStream;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.misc.*;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class BTreeLexer extends Lexer {
	static { RuntimeMetaData.checkVersion("4.8", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, OPERATOR=16, 
		BOP=17, FLOAT=18, WORD=19, WHITESPACE=20, NEWLINE=21, NL=22;
	public static String[] channelNames = {
		"DEFAULT_TOKEN_CHANNEL", "HIDDEN"
	};

	public static String[] modeNames = {
		"DEFAULT_MODE"
	};

	private static String[] makeRuleNames() {
		return new String[] {
			"T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6", "T__7", "T__8", 
			"T__9", "T__10", "T__11", "T__12", "T__13", "T__14", "OPERATOR", "BOP", 
			"FLOAT", "LOWERCASE", "UPPERCASE", "WORD", "WHITESPACE", "NEWLINE", "NL"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'BehaviorTree'", "':'", "'Tree'", "'('", "')'", "'['", "']'", 
			"'<'", "'>'", "'Condition'", "'Maneuver'", "'Subtree'", "'Key'", "'Params'", 
			"','"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, null, null, null, null, null, null, null, null, null, null, null, 
			null, null, null, null, "OPERATOR", "BOP", "FLOAT", "WORD", "WHITESPACE", 
			"NEWLINE", "NL"
		};
	}
	private static final String[] _SYMBOLIC_NAMES = makeSymbolicNames();
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}


	public BTreeLexer(CharStream input) {
		super(input);
		_interp = new LexerATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@Override
	public String getGrammarFileName() { return "BTree.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public String[] getChannelNames() { return channelNames; }

	@Override
	public String[] getModeNames() { return modeNames; }

	@Override
	public ATN getATN() { return _ATN; }

	public static final String _serializedATN =
		"\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2\30\u00c7\b\1\4\2"+
		"\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4"+
		"\13\t\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22"+
		"\t\22\4\23\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\4\30\t\30\4\31"+
		"\t\31\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\2\3\3\3\3\3\4"+
		"\3\4\3\4\3\4\3\4\3\5\3\5\3\6\3\6\3\7\3\7\3\b\3\b\3\t\3\t\3\n\3\n\3\13"+
		"\3\13\3\13\3\13\3\13\3\13\3\13\3\13\3\13\3\13\3\f\3\f\3\f\3\f\3\f\3\f"+
		"\3\f\3\f\3\f\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\16\3\16\3\16\3\16\3\17"+
		"\3\17\3\17\3\17\3\17\3\17\3\17\3\20\3\20\3\21\3\21\3\21\3\21\3\21\5\21"+
		"\u0081\n\21\3\22\3\22\3\22\3\22\3\22\3\22\3\22\3\22\3\22\3\22\3\22\3\22"+
		"\3\22\3\22\3\22\3\22\3\22\3\22\5\22\u0095\n\22\3\23\5\23\u0098\n\23\3"+
		"\23\7\23\u009b\n\23\f\23\16\23\u009e\13\23\3\23\5\23\u00a1\n\23\3\23\6"+
		"\23\u00a4\n\23\r\23\16\23\u00a5\3\24\3\24\3\25\3\25\3\26\3\26\3\26\6\26"+
		"\u00af\n\26\r\26\16\26\u00b0\3\27\3\27\3\30\5\30\u00b6\n\30\3\30\3\30"+
		"\6\30\u00ba\n\30\r\30\16\30\u00bb\3\31\5\31\u00bf\n\31\3\31\3\31\7\31"+
		"\u00c3\n\31\f\31\16\31\u00c6\13\31\2\2\32\3\3\5\4\7\5\t\6\13\7\r\b\17"+
		"\t\21\n\23\13\25\f\27\r\31\16\33\17\35\20\37\21!\22#\23%\24\'\2)\2+\25"+
		"-\26/\27\61\30\3\2\t\4\2>>@@\4\2--//\3\2\62;\3\2\60\60\3\2c|\3\2C\\\4"+
		"\2\13\13\"\"\2\u00d9\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3\2\2\2\2\t\3\2\2\2\2"+
		"\13\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2\2\2\21\3\2\2\2\2\23\3\2\2\2\2\25\3"+
		"\2\2\2\2\27\3\2\2\2\2\31\3\2\2\2\2\33\3\2\2\2\2\35\3\2\2\2\2\37\3\2\2"+
		"\2\2!\3\2\2\2\2#\3\2\2\2\2%\3\2\2\2\2+\3\2\2\2\2-\3\2\2\2\2/\3\2\2\2\2"+
		"\61\3\2\2\2\3\63\3\2\2\2\5@\3\2\2\2\7B\3\2\2\2\tG\3\2\2\2\13I\3\2\2\2"+
		"\rK\3\2\2\2\17M\3\2\2\2\21O\3\2\2\2\23Q\3\2\2\2\25S\3\2\2\2\27]\3\2\2"+
		"\2\31f\3\2\2\2\33n\3\2\2\2\35r\3\2\2\2\37y\3\2\2\2!\u0080\3\2\2\2#\u0094"+
		"\3\2\2\2%\u0097\3\2\2\2\'\u00a7\3\2\2\2)\u00a9\3\2\2\2+\u00ae\3\2\2\2"+
		"-\u00b2\3\2\2\2/\u00b9\3\2\2\2\61\u00be\3\2\2\2\63\64\7D\2\2\64\65\7g"+
		"\2\2\65\66\7j\2\2\66\67\7c\2\2\678\7x\2\289\7k\2\29:\7q\2\2:;\7t\2\2;"+
		"<\7V\2\2<=\7t\2\2=>\7g\2\2>?\7g\2\2?\4\3\2\2\2@A\7<\2\2A\6\3\2\2\2BC\7"+
		"V\2\2CD\7t\2\2DE\7g\2\2EF\7g\2\2F\b\3\2\2\2GH\7*\2\2H\n\3\2\2\2IJ\7+\2"+
		"\2J\f\3\2\2\2KL\7]\2\2L\16\3\2\2\2MN\7_\2\2N\20\3\2\2\2OP\7>\2\2P\22\3"+
		"\2\2\2QR\7@\2\2R\24\3\2\2\2ST\7E\2\2TU\7q\2\2UV\7p\2\2VW\7f\2\2WX\7k\2"+
		"\2XY\7v\2\2YZ\7k\2\2Z[\7q\2\2[\\\7p\2\2\\\26\3\2\2\2]^\7O\2\2^_\7c\2\2"+
		"_`\7p\2\2`a\7g\2\2ab\7w\2\2bc\7x\2\2cd\7g\2\2de\7t\2\2e\30\3\2\2\2fg\7"+
		"U\2\2gh\7w\2\2hi\7d\2\2ij\7v\2\2jk\7t\2\2kl\7g\2\2lm\7g\2\2m\32\3\2\2"+
		"\2no\7M\2\2op\7g\2\2pq\7{\2\2q\34\3\2\2\2rs\7R\2\2st\7c\2\2tu\7t\2\2u"+
		"v\7c\2\2vw\7o\2\2wx\7u\2\2x\36\3\2\2\2yz\7.\2\2z \3\2\2\2{\u0081\7A\2"+
		"\2|}\7/\2\2}\u0081\7@\2\2~\177\7?\2\2\177\u0081\7\63\2\2\u0080{\3\2\2"+
		"\2\u0080|\3\2\2\2\u0080~\3\2\2\2\u0081\"\3\2\2\2\u0082\u0095\t\2\2\2\u0083"+
		"\u0084\7?\2\2\u0084\u0095\7?\2\2\u0085\u0086\7@\2\2\u0086\u0095\7?\2\2"+
		"\u0087\u0088\7>\2\2\u0088\u0095\7?\2\2\u0089\u008a\7#\2\2\u008a\u0095"+
		"\7?\2\2\u008b\u0095\7?\2\2\u008c\u008d\7k\2\2\u008d\u0095\7u\2\2\u008e"+
		"\u008f\7k\2\2\u008f\u0090\7u\2\2\u0090\u0091\3\2\2\2\u0091\u0092\7p\2"+
		"\2\u0092\u0093\7q\2\2\u0093\u0095\7v\2\2\u0094\u0082\3\2\2\2\u0094\u0083"+
		"\3\2\2\2\u0094\u0085\3\2\2\2\u0094\u0087\3\2\2\2\u0094\u0089\3\2\2\2\u0094"+
		"\u008b\3\2\2\2\u0094\u008c\3\2\2\2\u0094\u008e\3\2\2\2\u0095$\3\2\2\2"+
		"\u0096\u0098\t\3\2\2\u0097\u0096\3\2\2\2\u0097\u0098\3\2\2\2\u0098\u00a0"+
		"\3\2\2\2\u0099\u009b\t\4\2\2\u009a\u0099\3\2\2\2\u009b\u009e\3\2\2\2\u009c"+
		"\u009a\3\2\2\2\u009c\u009d\3\2\2\2\u009d\u009f\3\2\2\2\u009e\u009c\3\2"+
		"\2\2\u009f\u00a1\t\5\2\2\u00a0\u009c\3\2\2\2\u00a0\u00a1\3\2\2\2\u00a1"+
		"\u00a3\3\2\2\2\u00a2\u00a4\t\4\2\2\u00a3\u00a2\3\2\2\2\u00a4\u00a5\3\2"+
		"\2\2\u00a5\u00a3\3\2\2\2\u00a5\u00a6\3\2\2\2\u00a6&\3\2\2\2\u00a7\u00a8"+
		"\t\6\2\2\u00a8(\3\2\2\2\u00a9\u00aa\t\7\2\2\u00aa*\3\2\2\2\u00ab\u00af"+
		"\5\'\24\2\u00ac\u00af\5)\25\2\u00ad\u00af\7a\2\2\u00ae\u00ab\3\2\2\2\u00ae"+
		"\u00ac\3\2\2\2\u00ae\u00ad\3\2\2\2\u00af\u00b0\3\2\2\2\u00b0\u00ae\3\2"+
		"\2\2\u00b0\u00b1\3\2\2\2\u00b1,\3\2\2\2\u00b2\u00b3\t\b\2\2\u00b3.\3\2"+
		"\2\2\u00b4\u00b6\7\17\2\2\u00b5\u00b4\3\2\2\2\u00b5\u00b6\3\2\2\2\u00b6"+
		"\u00b7\3\2\2\2\u00b7\u00ba\7\f\2\2\u00b8\u00ba\7\17\2\2\u00b9\u00b5\3"+
		"\2\2\2\u00b9\u00b8\3\2\2\2\u00ba\u00bb\3\2\2\2\u00bb\u00b9\3\2\2\2\u00bb"+
		"\u00bc\3\2\2\2\u00bc\60\3\2\2\2\u00bd\u00bf\7\17\2\2\u00be\u00bd\3\2\2"+
		"\2\u00be\u00bf\3\2\2\2\u00bf\u00c0\3\2\2\2\u00c0\u00c4\7\f\2\2\u00c1\u00c3"+
		"\7\"\2\2\u00c2\u00c1\3\2\2\2\u00c3\u00c6\3\2\2\2\u00c4\u00c2\3\2\2\2\u00c4"+
		"\u00c5\3\2\2\2\u00c5\62\3\2\2\2\u00c6\u00c4\3\2\2\2\20\2\u0080\u0094\u0097"+
		"\u009c\u00a0\u00a5\u00ae\u00b0\u00b5\u00b9\u00bb\u00be\u00c4\2";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}