// Generated from /home/rcaldas/phd/research/explore_research_topic/behavior_trees/impl/geoscenarioserver/scenarios/btree_parser/BTreeDSL.g4 by ANTLR 4.8
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class BTreeDSLParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.8", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		OPERATOR=10, BOP=11, ATT=12, FLOAT=13, WORD=14, WS=15, NL=16, INDENT=17, 
		DEDENT=18;
	public static final int
		RULE_behavior_tree = 0, RULE_root_node = 1, RULE_node = 2, RULE_node_composition = 3, 
		RULE_leaf_node = 4, RULE_condition = 5, RULE_maneuver = 6, RULE_subtree = 7, 
		RULE_key = 8, RULE_params = 9, RULE_bexpr = 10, RULE_value = 11, RULE_name = 12;
	private static String[] makeRuleNames() {
		return new String[] {
			"behavior_tree", "root_node", "node", "node_composition", "leaf_node", 
			"condition", "maneuver", "subtree", "key", "params", "bexpr", "value", 
			"name"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'BehaviorTree'", "':'", "'Condition'", "'('", "')'", "'Maneuver'", 
			"','", "'Subtree'", "'key'", null, null, "'='"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, null, null, null, null, null, null, null, null, null, "OPERATOR", 
			"BOP", "ATT", "FLOAT", "WORD", "WS", "NL", "INDENT", "DEDENT"
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
	public String getGrammarFileName() { return "BTreeDSL.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public BTreeDSLParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	public static class Behavior_treeContext extends ParserRuleContext {
		public TerminalNode EOF() { return getToken(BTreeDSLParser.EOF, 0); }
		public List<NameContext> name() {
			return getRuleContexts(NameContext.class);
		}
		public NameContext name(int i) {
			return getRuleContext(NameContext.class,i);
		}
		public List<TerminalNode> INDENT() { return getTokens(BTreeDSLParser.INDENT); }
		public TerminalNode INDENT(int i) {
			return getToken(BTreeDSLParser.INDENT, i);
		}
		public List<Root_nodeContext> root_node() {
			return getRuleContexts(Root_nodeContext.class);
		}
		public Root_nodeContext root_node(int i) {
			return getRuleContext(Root_nodeContext.class,i);
		}
		public List<TerminalNode> NL() { return getTokens(BTreeDSLParser.NL); }
		public TerminalNode NL(int i) {
			return getToken(BTreeDSLParser.NL, i);
		}
		public List<TerminalNode> DEDENT() { return getTokens(BTreeDSLParser.DEDENT); }
		public TerminalNode DEDENT(int i) {
			return getToken(BTreeDSLParser.DEDENT, i);
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
			setState(37); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(26);
				match(T__0);
				setState(27);
				name();
				setState(28);
				match(T__1);
				setState(29);
				match(INDENT);
				setState(30);
				root_node();
				setState(32);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==NL) {
					{
					setState(31);
					match(NL);
					}
				}

				setState(35);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==DEDENT) {
					{
					setState(34);
					match(DEDENT);
					}
				}

				}
				}
				setState(39); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==T__0 );
			setState(41);
			match(EOF);
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

	public static class Root_nodeContext extends ParserRuleContext {
		public NodeContext node() {
			return getRuleContext(NodeContext.class,0);
		}
		public Root_nodeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_root_node; }
	}

	public final Root_nodeContext root_node() throws RecognitionException {
		Root_nodeContext _localctx = new Root_nodeContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_root_node);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(43);
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
		public Leaf_nodeContext leaf_node() {
			return getRuleContext(Leaf_nodeContext.class,0);
		}
		public Node_compositionContext node_composition() {
			return getRuleContext(Node_compositionContext.class,0);
		}
		public NodeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_node; }
	}

	public final NodeContext node() throws RecognitionException {
		NodeContext _localctx = new NodeContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_node);
		try {
			setState(47);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__2:
			case T__5:
			case T__7:
				enterOuterAlt(_localctx, 1);
				{
				setState(45);
				leaf_node();
				}
				break;
			case OPERATOR:
				enterOuterAlt(_localctx, 2);
				{
				setState(46);
				node_composition();
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

	public static class Node_compositionContext extends ParserRuleContext {
		public TerminalNode OPERATOR() { return getToken(BTreeDSLParser.OPERATOR, 0); }
		public TerminalNode INDENT() { return getToken(BTreeDSLParser.INDENT, 0); }
		public TerminalNode DEDENT() { return getToken(BTreeDSLParser.DEDENT, 0); }
		public List<NodeContext> node() {
			return getRuleContexts(NodeContext.class);
		}
		public NodeContext node(int i) {
			return getRuleContext(NodeContext.class,i);
		}
		public Node_compositionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_node_composition; }
	}

	public final Node_compositionContext node_composition() throws RecognitionException {
		Node_compositionContext _localctx = new Node_compositionContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_node_composition);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(49);
			match(OPERATOR);
			setState(50);
			match(INDENT);
			setState(52); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(51);
				node();
				}
				}
				setState(54); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & ((1L << T__2) | (1L << T__5) | (1L << T__7) | (1L << OPERATOR))) != 0) );
			setState(56);
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
		public TerminalNode NL() { return getToken(BTreeDSLParser.NL, 0); }
		public ManeuverContext maneuver() {
			return getRuleContext(ManeuverContext.class,0);
		}
		public ConditionContext condition() {
			return getRuleContext(ConditionContext.class,0);
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
		enterRule(_localctx, 8, RULE_leaf_node);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(61);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__5:
				{
				setState(58);
				maneuver();
				}
				break;
			case T__2:
				{
				setState(59);
				condition();
				}
				break;
			case T__7:
				{
				setState(60);
				subtree();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(63);
			match(NL);
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
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public ParamsContext params() {
			return getRuleContext(ParamsContext.class,0);
		}
		public ConditionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_condition; }
	}

	public final ConditionContext condition() throws RecognitionException {
		ConditionContext _localctx = new ConditionContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_condition);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(65);
			match(T__2);
			setState(66);
			name();
			setState(71);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(67);
				match(T__3);
				setState(68);
				params();
				setState(69);
				match(T__4);
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

	public static class ManeuverContext extends ParserRuleContext {
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public KeyContext key() {
			return getRuleContext(KeyContext.class,0);
		}
		public List<ParamsContext> params() {
			return getRuleContexts(ParamsContext.class);
		}
		public ParamsContext params(int i) {
			return getRuleContext(ParamsContext.class,i);
		}
		public ManeuverContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_maneuver; }
	}

	public final ManeuverContext maneuver() throws RecognitionException {
		ManeuverContext _localctx = new ManeuverContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_maneuver);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(73);
			match(T__5);
			setState(74);
			name();
			setState(86);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(75);
				match(T__3);
				setState(76);
				key();
				setState(81);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__6) {
					{
					{
					setState(77);
					match(T__6);
					setState(78);
					params();
					}
					}
					setState(83);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(84);
				match(T__4);
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

	public static class SubtreeContext extends ParserRuleContext {
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public ParamsContext params() {
			return getRuleContext(ParamsContext.class,0);
		}
		public SubtreeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_subtree; }
	}

	public final SubtreeContext subtree() throws RecognitionException {
		SubtreeContext _localctx = new SubtreeContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_subtree);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(88);
			match(T__7);
			setState(89);
			name();
			setState(94);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(90);
				match(T__3);
				setState(91);
				params();
				setState(92);
				match(T__4);
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

	public static class KeyContext extends ParserRuleContext {
		public TerminalNode ATT() { return getToken(BTreeDSLParser.ATT, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public KeyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_key; }
	}

	public final KeyContext key() throws RecognitionException {
		KeyContext _localctx = new KeyContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_key);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(96);
			match(T__8);
			setState(97);
			match(ATT);
			setState(98);
			name();
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
		enterRule(_localctx, 18, RULE_params);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(100);
			bexpr();
			setState(105);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,10,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(101);
					match(T__6);
					setState(102);
					bexpr();
					}
					} 
				}
				setState(107);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,10,_ctx);
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
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public TerminalNode BOP() { return getToken(BTreeDSLParser.BOP, 0); }
		public TerminalNode ATT() { return getToken(BTreeDSLParser.ATT, 0); }
		public BexprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bexpr; }
	}

	public final BexprContext bexpr() throws RecognitionException {
		BexprContext _localctx = new BexprContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_bexpr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(108);
			name();
			setState(109);
			_la = _input.LA(1);
			if ( !(_la==BOP || _la==ATT) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			setState(110);
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
		public TerminalNode FLOAT() { return getToken(BTreeDSLParser.FLOAT, 0); }
		public NameContext name() {
			return getRuleContext(NameContext.class,0);
		}
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_value);
		try {
			setState(114);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case FLOAT:
				enterOuterAlt(_localctx, 1);
				{
				setState(112);
				match(FLOAT);
				}
				break;
			case WORD:
			case WS:
				enterOuterAlt(_localctx, 2);
				{
				setState(113);
				name();
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

	public static class NameContext extends ParserRuleContext {
		public TerminalNode WORD() { return getToken(BTreeDSLParser.WORD, 0); }
		public List<TerminalNode> WS() { return getTokens(BTreeDSLParser.WS); }
		public TerminalNode WS(int i) {
			return getToken(BTreeDSLParser.WS, i);
		}
		public NameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_name; }
	}

	public final NameContext name() throws RecognitionException {
		NameContext _localctx = new NameContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_name);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(119);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==WS) {
				{
				{
				setState(116);
				match(WS);
				}
				}
				setState(121);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(122);
			match(WORD);
			setState(126);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==WS) {
				{
				{
				setState(123);
				match(WS);
				}
				}
				setState(128);
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

	public static final String _serializedATN =
		"\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\3\24\u0084\4\2\t\2"+
		"\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13"+
		"\t\13\4\f\t\f\4\r\t\r\4\16\t\16\3\2\3\2\3\2\3\2\3\2\3\2\5\2#\n\2\3\2\5"+
		"\2&\n\2\6\2(\n\2\r\2\16\2)\3\2\3\2\3\3\3\3\3\4\3\4\5\4\62\n\4\3\5\3\5"+
		"\3\5\6\5\67\n\5\r\5\16\58\3\5\3\5\3\6\3\6\3\6\5\6@\n\6\3\6\3\6\3\7\3\7"+
		"\3\7\3\7\3\7\3\7\5\7J\n\7\3\b\3\b\3\b\3\b\3\b\3\b\7\bR\n\b\f\b\16\bU\13"+
		"\b\3\b\3\b\5\bY\n\b\3\t\3\t\3\t\3\t\3\t\3\t\5\ta\n\t\3\n\3\n\3\n\3\n\3"+
		"\13\3\13\3\13\7\13j\n\13\f\13\16\13m\13\13\3\f\3\f\3\f\3\f\3\r\3\r\5\r"+
		"u\n\r\3\16\7\16x\n\16\f\16\16\16{\13\16\3\16\3\16\7\16\177\n\16\f\16\16"+
		"\16\u0082\13\16\3\16\2\2\17\2\4\6\b\n\f\16\20\22\24\26\30\32\2\3\3\2\r"+
		"\16\2\u0085\2\'\3\2\2\2\4-\3\2\2\2\6\61\3\2\2\2\b\63\3\2\2\2\n?\3\2\2"+
		"\2\fC\3\2\2\2\16K\3\2\2\2\20Z\3\2\2\2\22b\3\2\2\2\24f\3\2\2\2\26n\3\2"+
		"\2\2\30t\3\2\2\2\32y\3\2\2\2\34\35\7\3\2\2\35\36\5\32\16\2\36\37\7\4\2"+
		"\2\37 \7\23\2\2 \"\5\4\3\2!#\7\22\2\2\"!\3\2\2\2\"#\3\2\2\2#%\3\2\2\2"+
		"$&\7\24\2\2%$\3\2\2\2%&\3\2\2\2&(\3\2\2\2\'\34\3\2\2\2()\3\2\2\2)\'\3"+
		"\2\2\2)*\3\2\2\2*+\3\2\2\2+,\7\2\2\3,\3\3\2\2\2-.\5\6\4\2.\5\3\2\2\2/"+
		"\62\5\n\6\2\60\62\5\b\5\2\61/\3\2\2\2\61\60\3\2\2\2\62\7\3\2\2\2\63\64"+
		"\7\f\2\2\64\66\7\23\2\2\65\67\5\6\4\2\66\65\3\2\2\2\678\3\2\2\28\66\3"+
		"\2\2\289\3\2\2\29:\3\2\2\2:;\7\24\2\2;\t\3\2\2\2<@\5\16\b\2=@\5\f\7\2"+
		">@\5\20\t\2?<\3\2\2\2?=\3\2\2\2?>\3\2\2\2@A\3\2\2\2AB\7\22\2\2B\13\3\2"+
		"\2\2CD\7\5\2\2DI\5\32\16\2EF\7\6\2\2FG\5\24\13\2GH\7\7\2\2HJ\3\2\2\2I"+
		"E\3\2\2\2IJ\3\2\2\2J\r\3\2\2\2KL\7\b\2\2LX\5\32\16\2MN\7\6\2\2NS\5\22"+
		"\n\2OP\7\t\2\2PR\5\24\13\2QO\3\2\2\2RU\3\2\2\2SQ\3\2\2\2ST\3\2\2\2TV\3"+
		"\2\2\2US\3\2\2\2VW\7\7\2\2WY\3\2\2\2XM\3\2\2\2XY\3\2\2\2Y\17\3\2\2\2Z"+
		"[\7\n\2\2[`\5\32\16\2\\]\7\6\2\2]^\5\24\13\2^_\7\7\2\2_a\3\2\2\2`\\\3"+
		"\2\2\2`a\3\2\2\2a\21\3\2\2\2bc\7\13\2\2cd\7\16\2\2de\5\32\16\2e\23\3\2"+
		"\2\2fk\5\26\f\2gh\7\t\2\2hj\5\26\f\2ig\3\2\2\2jm\3\2\2\2ki\3\2\2\2kl\3"+
		"\2\2\2l\25\3\2\2\2mk\3\2\2\2no\5\32\16\2op\t\2\2\2pq\5\30\r\2q\27\3\2"+
		"\2\2ru\7\17\2\2su\5\32\16\2tr\3\2\2\2ts\3\2\2\2u\31\3\2\2\2vx\7\21\2\2"+
		"wv\3\2\2\2x{\3\2\2\2yw\3\2\2\2yz\3\2\2\2z|\3\2\2\2{y\3\2\2\2|\u0080\7"+
		"\20\2\2}\177\7\21\2\2~}\3\2\2\2\177\u0082\3\2\2\2\u0080~\3\2\2\2\u0080"+
		"\u0081\3\2\2\2\u0081\33\3\2\2\2\u0082\u0080\3\2\2\2\20\"%)\618?ISX`kt"+
		"y\u0080";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}