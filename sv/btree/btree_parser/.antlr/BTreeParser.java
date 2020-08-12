// Generated from /home/rcaldas/phd/research/explore_research_topic/behavior_trees/impl/geoscenarioserver/scenarios/btree_parser/BTree.g4 by ANTLR 4.8
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class BTreeParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.8", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, OPERATOR=16, 
		BOP=17, FLOAT=18, WORD=19, WHITESPACE=20, NEWLINE=21, NL=22, INDENT=23, 
		DEDENT=24;
	public static final int
		RULE_behavior_tree = 0, RULE_define_tree = 1, RULE_node = 2, RULE_leaf_node = 3, 
		RULE_condition = 4, RULE_maneuver = 5, RULE_subtree = 6, RULE_define_condition = 7, 
		RULE_define_maneuver = 8, RULE_define_subtree = 9, RULE_key = 10, RULE_params = 11, 
		RULE_bexpr = 12, RULE_value = 13;
	private static String[] makeRuleNames() {
		return new String[] {
			"behavior_tree", "define_tree", "node", "leaf_node", "condition", "maneuver", 
			"subtree", "define_condition", "define_maneuver", "define_subtree", "key", 
			"params", "bexpr", "value"
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
			"NEWLINE", "NL", "INDENT", "DEDENT"
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

	@Override
	public String getGrammarFileName() { return "BTree.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public BTreeParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	public static class Behavior_treeContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public List<TerminalNode> NEWLINE() { return getTokens(BTreeParser.NEWLINE); }
		public TerminalNode NEWLINE(int i) {
			return getToken(BTreeParser.NEWLINE, i);
		}
		public List<TerminalNode> INDENT() { return getTokens(BTreeParser.INDENT); }
		public TerminalNode INDENT(int i) {
			return getToken(BTreeParser.INDENT, i);
		}
		public Define_treeContext define_tree() {
			return getRuleContext(Define_treeContext.class,0);
		}
		public List<TerminalNode> DEDENT() { return getTokens(BTreeParser.DEDENT); }
		public TerminalNode DEDENT(int i) {
			return getToken(BTreeParser.DEDENT, i);
		}
		public List<Define_conditionContext> define_condition() {
			return getRuleContexts(Define_conditionContext.class);
		}
		public Define_conditionContext define_condition(int i) {
			return getRuleContext(Define_conditionContext.class,i);
		}
		public List<Define_maneuverContext> define_maneuver() {
			return getRuleContexts(Define_maneuverContext.class);
		}
		public Define_maneuverContext define_maneuver(int i) {
			return getRuleContext(Define_maneuverContext.class,i);
		}
		public List<Define_subtreeContext> define_subtree() {
			return getRuleContexts(Define_subtreeContext.class);
		}
		public Define_subtreeContext define_subtree(int i) {
			return getRuleContext(Define_subtreeContext.class,i);
		}
		public Behavior_treeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_behavior_tree; }
	}

	public final Behavior_treeContext behavior_tree() throws RecognitionException {
		Behavior_treeContext _localctx = new Behavior_treeContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_behavior_tree);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(28);
			match(T__0);
			setState(29);
			match(WORD);
			setState(30);
			match(T__1);
			setState(31);
			match(NEWLINE);
			setState(32);
			match(INDENT);
			setState(33);
			define_tree();
			setState(34);
			match(DEDENT);
			setState(35);
			match(NEWLINE);
			setState(36);
			match(INDENT);
			setState(40);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__9) {
				{
				{
				setState(37);
				define_condition();
				}
				}
				setState(42);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(43);
			match(DEDENT);
			setState(44);
			match(NEWLINE);
			setState(45);
			match(INDENT);
			setState(49);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__10) {
				{
				{
				setState(46);
				define_maneuver();
				}
				}
				setState(51);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(52);
			match(DEDENT);
			setState(53);
			match(NEWLINE);
			setState(54);
			match(INDENT);
			setState(58);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__11) {
				{
				{
				setState(55);
				define_subtree();
				}
				}
				setState(60);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(61);
			match(DEDENT);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Define_treeContext extends ParserRuleContext {
		public NodeContext node() {
			return getRuleContext(NodeContext.class,0);
		}
		public Define_treeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_define_tree; }
	}

	public final Define_treeContext define_tree() throws RecognitionException {
		Define_treeContext _localctx = new Define_treeContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_define_tree);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(63);
			match(T__2);
			setState(64);
			match(T__1);
			setState(65);
			node();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class NodeContext extends ParserRuleContext {
		public TerminalNode NEWLINE() { return getToken(BTreeParser.NEWLINE, 0); }
		public TerminalNode INDENT() { return getToken(BTreeParser.INDENT, 0); }
		public TerminalNode DEDENT() { return getToken(BTreeParser.DEDENT, 0); }
		public Leaf_nodeContext leaf_node() {
			return getRuleContext(Leaf_nodeContext.class,0);
		}
		public TerminalNode OPERATOR() { return getToken(BTreeParser.OPERATOR, 0); }
		public List<NodeContext> node() {
			return getRuleContexts(NodeContext.class);
		}
		public NodeContext node(int i) {
			return getRuleContext(NodeContext.class,i);
		}
		public NodeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_node; }
	}

	public final NodeContext node() throws RecognitionException {
		NodeContext _localctx = new NodeContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_node);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(67);
			match(NEWLINE);
			setState(68);
			match(INDENT);
			setState(78);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__3:
			case T__5:
			case T__7:
				{
				setState(69);
				leaf_node();
				}
				break;
			case OPERATOR:
				{
				setState(70);
				match(OPERATOR);
				setState(71);
				node();
				setState(75);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==NEWLINE) {
					{
					{
					setState(72);
					node();
					}
					}
					setState(77);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(80);
			match(DEDENT);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Leaf_nodeContext extends ParserRuleContext {
		public ConditionContext condition() {
			return getRuleContext(ConditionContext.class,0);
		}
		public ManeuverContext maneuver() {
			return getRuleContext(ManeuverContext.class,0);
		}
		public SubtreeContext subtree() {
			return getRuleContext(SubtreeContext.class,0);
		}
		public Leaf_nodeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_leaf_node; }
	}

	public final Leaf_nodeContext leaf_node() throws RecognitionException {
		Leaf_nodeContext _localctx = new Leaf_nodeContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_leaf_node);
		try {
			setState(85);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__3:
				enterOuterAlt(_localctx, 1);
				{
				setState(82);
				condition();
				}
				break;
			case T__5:
				enterOuterAlt(_localctx, 2);
				{
				setState(83);
				maneuver();
				}
				break;
			case T__7:
				enterOuterAlt(_localctx, 3);
				{
				setState(84);
				subtree();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ConditionContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public ConditionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_condition; }
	}

	public final ConditionContext condition() throws RecognitionException {
		ConditionContext _localctx = new ConditionContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_condition);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(87);
			match(T__3);
			setState(88);
			match(WORD);
			setState(89);
			match(T__4);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ManeuverContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public ManeuverContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_maneuver; }
	}

	public final ManeuverContext maneuver() throws RecognitionException {
		ManeuverContext _localctx = new ManeuverContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_maneuver);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(91);
			match(T__5);
			setState(92);
			match(WORD);
			setState(93);
			match(T__6);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class SubtreeContext extends ParserRuleContext {
		public SubtreeContext subtree() {
			return getRuleContext(SubtreeContext.class,0);
		}
		public SubtreeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtree; }
	}

	public final SubtreeContext subtree() throws RecognitionException {
		SubtreeContext _localctx = new SubtreeContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_subtree);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(95);
			match(T__7);
			setState(96);
			subtree();
			setState(97);
			match(T__8);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Define_conditionContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public List<ParamsContext> params() {
			return getRuleContexts(ParamsContext.class);
		}
		public ParamsContext params(int i) {
			return getRuleContext(ParamsContext.class,i);
		}
		public Define_conditionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_define_condition; }
	}

	public final Define_conditionContext define_condition() throws RecognitionException {
		Define_conditionContext _localctx = new Define_conditionContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_define_condition);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(99);
			match(T__9);
			setState(100);
			match(WORD);
			setState(101);
			match(T__1);
			setState(105);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__13) {
				{
				{
				setState(102);
				params();
				}
				}
				setState(107);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Define_maneuverContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public KeyContext key() {
			return getRuleContext(KeyContext.class,0);
		}
		public List<ParamsContext> params() {
			return getRuleContexts(ParamsContext.class);
		}
		public ParamsContext params(int i) {
			return getRuleContext(ParamsContext.class,i);
		}
		public Define_maneuverContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_define_maneuver; }
	}

	public final Define_maneuverContext define_maneuver() throws RecognitionException {
		Define_maneuverContext _localctx = new Define_maneuverContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_define_maneuver);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(108);
			match(T__10);
			setState(109);
			match(WORD);
			setState(110);
			match(T__1);
			setState(111);
			key();
			setState(112);
			match(T__1);
			setState(116);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__13) {
				{
				{
				setState(113);
				params();
				}
				}
				setState(118);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Define_subtreeContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public Define_subtreeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_define_subtree; }
	}

	public final Define_subtreeContext define_subtree() throws RecognitionException {
		Define_subtreeContext _localctx = new Define_subtreeContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_define_subtree);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(119);
			match(T__11);
			setState(120);
			match(WORD);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class KeyContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public KeyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_key; }
	}

	public final KeyContext key() throws RecognitionException {
		KeyContext _localctx = new KeyContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_key);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(122);
			match(T__12);
			setState(123);
			match(T__1);
			setState(124);
			match(WORD);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ParamsContext extends ParserRuleContext {
		public List<BexprContext> bexpr() {
			return getRuleContexts(BexprContext.class);
		}
		public BexprContext bexpr(int i) {
			return getRuleContext(BexprContext.class,i);
		}
		public ParamsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_params; }
	}

	public final ParamsContext params() throws RecognitionException {
		ParamsContext _localctx = new ParamsContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_params);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(126);
			match(T__13);
			setState(127);
			match(T__1);
			setState(136);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==WORD) {
				{
				setState(128);
				bexpr();
				setState(133);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__14) {
					{
					{
					setState(129);
					match(T__14);
					setState(130);
					bexpr();
					}
					}
					setState(135);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class BexprContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public TerminalNode BOP() { return getToken(BTreeParser.BOP, 0); }
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public BexprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bexpr; }
	}

	public final BexprContext bexpr() throws RecognitionException {
		BexprContext _localctx = new BexprContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_bexpr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(138);
			match(WORD);
			setState(139);
			match(BOP);
			setState(140);
			value();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ValueContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeParser.WORD, 0); }
		public TerminalNode FLOAT() { return getToken(BTreeParser.FLOAT, 0); }
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_value);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(142);
			_la = _input.LA(1);
			if ( !(_la==FLOAT || _la==WORD) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\3\32\u0093\4\2\t\2"+
		"\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13"+
		"\t\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\3\2\3\2\3\2\3\2\3\2\3\2\3\2"+
		"\3\2\3\2\3\2\7\2)\n\2\f\2\16\2,\13\2\3\2\3\2\3\2\3\2\7\2\62\n\2\f\2\16"+
		"\2\65\13\2\3\2\3\2\3\2\3\2\7\2;\n\2\f\2\16\2>\13\2\3\2\3\2\3\3\3\3\3\3"+
		"\3\3\3\4\3\4\3\4\3\4\3\4\3\4\7\4L\n\4\f\4\16\4O\13\4\5\4Q\n\4\3\4\3\4"+
		"\3\5\3\5\3\5\5\5X\n\5\3\6\3\6\3\6\3\6\3\7\3\7\3\7\3\7\3\b\3\b\3\b\3\b"+
		"\3\t\3\t\3\t\3\t\7\tj\n\t\f\t\16\tm\13\t\3\n\3\n\3\n\3\n\3\n\3\n\7\nu"+
		"\n\n\f\n\16\nx\13\n\3\13\3\13\3\13\3\f\3\f\3\f\3\f\3\r\3\r\3\r\3\r\3\r"+
		"\7\r\u0086\n\r\f\r\16\r\u0089\13\r\5\r\u008b\n\r\3\16\3\16\3\16\3\16\3"+
		"\17\3\17\3\17\2\2\20\2\4\6\b\n\f\16\20\22\24\26\30\32\34\2\3\3\2\24\25"+
		"\2\u008f\2\36\3\2\2\2\4A\3\2\2\2\6E\3\2\2\2\bW\3\2\2\2\nY\3\2\2\2\f]\3"+
		"\2\2\2\16a\3\2\2\2\20e\3\2\2\2\22n\3\2\2\2\24y\3\2\2\2\26|\3\2\2\2\30"+
		"\u0080\3\2\2\2\32\u008c\3\2\2\2\34\u0090\3\2\2\2\36\37\7\3\2\2\37 \7\25"+
		"\2\2 !\7\4\2\2!\"\7\27\2\2\"#\7\31\2\2#$\5\4\3\2$%\7\32\2\2%&\7\27\2\2"+
		"&*\7\31\2\2\')\5\20\t\2(\'\3\2\2\2),\3\2\2\2*(\3\2\2\2*+\3\2\2\2+-\3\2"+
		"\2\2,*\3\2\2\2-.\7\32\2\2./\7\27\2\2/\63\7\31\2\2\60\62\5\22\n\2\61\60"+
		"\3\2\2\2\62\65\3\2\2\2\63\61\3\2\2\2\63\64\3\2\2\2\64\66\3\2\2\2\65\63"+
		"\3\2\2\2\66\67\7\32\2\2\678\7\27\2\28<\7\31\2\29;\5\24\13\2:9\3\2\2\2"+
		";>\3\2\2\2<:\3\2\2\2<=\3\2\2\2=?\3\2\2\2><\3\2\2\2?@\7\32\2\2@\3\3\2\2"+
		"\2AB\7\5\2\2BC\7\4\2\2CD\5\6\4\2D\5\3\2\2\2EF\7\27\2\2FP\7\31\2\2GQ\5"+
		"\b\5\2HI\7\22\2\2IM\5\6\4\2JL\5\6\4\2KJ\3\2\2\2LO\3\2\2\2MK\3\2\2\2MN"+
		"\3\2\2\2NQ\3\2\2\2OM\3\2\2\2PG\3\2\2\2PH\3\2\2\2QR\3\2\2\2RS\7\32\2\2"+
		"S\7\3\2\2\2TX\5\n\6\2UX\5\f\7\2VX\5\16\b\2WT\3\2\2\2WU\3\2\2\2WV\3\2\2"+
		"\2X\t\3\2\2\2YZ\7\6\2\2Z[\7\25\2\2[\\\7\7\2\2\\\13\3\2\2\2]^\7\b\2\2^"+
		"_\7\25\2\2_`\7\t\2\2`\r\3\2\2\2ab\7\n\2\2bc\5\16\b\2cd\7\13\2\2d\17\3"+
		"\2\2\2ef\7\f\2\2fg\7\25\2\2gk\7\4\2\2hj\5\30\r\2ih\3\2\2\2jm\3\2\2\2k"+
		"i\3\2\2\2kl\3\2\2\2l\21\3\2\2\2mk\3\2\2\2no\7\r\2\2op\7\25\2\2pq\7\4\2"+
		"\2qr\5\26\f\2rv\7\4\2\2su\5\30\r\2ts\3\2\2\2ux\3\2\2\2vt\3\2\2\2vw\3\2"+
		"\2\2w\23\3\2\2\2xv\3\2\2\2yz\7\16\2\2z{\7\25\2\2{\25\3\2\2\2|}\7\17\2"+
		"\2}~\7\4\2\2~\177\7\25\2\2\177\27\3\2\2\2\u0080\u0081\7\20\2\2\u0081\u008a"+
		"\7\4\2\2\u0082\u0087\5\32\16\2\u0083\u0084\7\21\2\2\u0084\u0086\5\32\16"+
		"\2\u0085\u0083\3\2\2\2\u0086\u0089\3\2\2\2\u0087\u0085\3\2\2\2\u0087\u0088"+
		"\3\2\2\2\u0088\u008b\3\2\2\2\u0089\u0087\3\2\2\2\u008a\u0082\3\2\2\2\u008a"+
		"\u008b\3\2\2\2\u008b\31\3\2\2\2\u008c\u008d\7\25\2\2\u008d\u008e\7\23"+
		"\2\2\u008e\u008f\5\34\17\2\u008f\33\3\2\2\2\u0090\u0091\t\2\2\2\u0091"+
		"\35\3\2\2\2\f*\63<MPWkv\u0087\u008a";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}