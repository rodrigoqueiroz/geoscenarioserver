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
		RULE_behavior_tree = 0, RULE_stmt = 1, RULE_block_stmt = 2, RULE_simple_stmt = 3, 
		RULE_condition = 4, RULE_maneuver = 5, RULE_subtree = 6, RULE_key = 7, 
		RULE_params = 8, RULE_bexpr = 9, RULE_value = 10, RULE_name = 11;
	private static String[] makeRuleNames() {
		return new String[] {
			"behavior_tree", "stmt", "block_stmt", "simple_stmt", "condition", "maneuver", 
			"subtree", "key", "params", "bexpr", "value", "name"
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
		public List<StmtContext> stmt() {
			return getRuleContexts(StmtContext.class);
		}
		public StmtContext stmt(int i) {
			return getRuleContext(StmtContext.class,i);
		}
		public List<ConditionContext> condition() {
			return getRuleContexts(ConditionContext.class);
		}
		public ConditionContext condition(int i) {
			return getRuleContext(ConditionContext.class,i);
		}
		public List<ManeuverContext> maneuver() {
			return getRuleContexts(ManeuverContext.class);
		}
		public ManeuverContext maneuver(int i) {
			return getRuleContext(ManeuverContext.class,i);
		}
		public List<SubtreeContext> subtree() {
			return getRuleContexts(SubtreeContext.class);
		}
		public SubtreeContext subtree(int i) {
			return getRuleContext(SubtreeContext.class,i);
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
			setState(53); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(24);
				match(T__0);
				setState(25);
				name();
				setState(26);
				match(T__1);
				setState(27);
				match(INDENT);
				setState(28);
				stmt();
				setState(32);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__2) {
					{
					{
					setState(29);
					condition();
					}
					}
					setState(34);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(38);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__5) {
					{
					{
					setState(35);
					maneuver();
					}
					}
					setState(40);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(44);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__7) {
					{
					{
					setState(41);
					subtree();
					}
					}
					setState(46);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(48);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==NL) {
					{
					setState(47);
					match(NL);
					}
				}

				setState(51);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==DEDENT) {
					{
					setState(50);
					match(DEDENT);
					}
				}

				}
				}
				setState(55); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==T__0 );
			setState(57);
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

	public static class StmtContext extends ParserRuleContext {
		public Simple_stmtContext simple_stmt() {
			return getRuleContext(Simple_stmtContext.class,0);
		}
		public Block_stmtContext block_stmt() {
			return getRuleContext(Block_stmtContext.class,0);
		}
		public StmtContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_stmt; }
	}

	public final StmtContext stmt() throws RecognitionException {
		StmtContext _localctx = new StmtContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_stmt);
		try {
			setState(61);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__2:
			case T__5:
			case T__7:
				enterOuterAlt(_localctx, 1);
				{
				setState(59);
				simple_stmt();
				}
				break;
			case OPERATOR:
				enterOuterAlt(_localctx, 2);
				{
				setState(60);
				block_stmt();
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

	public static class Block_stmtContext extends ParserRuleContext {
		public TerminalNode OPERATOR() { return getToken(BTreeDSLParser.OPERATOR, 0); }
		public TerminalNode INDENT() { return getToken(BTreeDSLParser.INDENT, 0); }
		public TerminalNode DEDENT() { return getToken(BTreeDSLParser.DEDENT, 0); }
		public List<StmtContext> stmt() {
			return getRuleContexts(StmtContext.class);
		}
		public StmtContext stmt(int i) {
			return getRuleContext(StmtContext.class,i);
		}
		public Block_stmtContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_block_stmt; }
	}

	public final Block_stmtContext block_stmt() throws RecognitionException {
		Block_stmtContext _localctx = new Block_stmtContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_block_stmt);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(63);
			match(OPERATOR);
			setState(64);
			match(INDENT);
			setState(66); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(65);
				stmt();
				}
				}
				setState(68); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & ((1L << T__2) | (1L << T__5) | (1L << T__7) | (1L << OPERATOR))) != 0) );
			setState(70);
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

	public static class Simple_stmtContext extends ParserRuleContext {
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
		public Simple_stmtContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_simple_stmt; }
	}

	public final Simple_stmtContext simple_stmt() throws RecognitionException {
		Simple_stmtContext _localctx = new Simple_stmtContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_simple_stmt);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(75);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__5:
				{
				setState(72);
				maneuver();
				}
				break;
			case T__2:
				{
				setState(73);
				condition();
				}
				break;
			case T__7:
				{
				setState(74);
				subtree();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(77);
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
		enterRule(_localctx, 8, RULE_condition);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(79);
			match(T__2);
			setState(80);
			name();
			setState(85);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(81);
				match(T__3);
				setState(82);
				params();
				setState(83);
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
		enterRule(_localctx, 10, RULE_maneuver);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(87);
			match(T__5);
			setState(88);
			name();
			setState(100);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(89);
				match(T__3);
				setState(90);
				key();
				setState(95);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==T__6) {
					{
					{
					setState(91);
					match(T__6);
					setState(92);
					params();
					}
					}
					setState(97);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				setState(98);
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
		enterRule(_localctx, 12, RULE_subtree);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(102);
			match(T__7);
			setState(103);
			name();
			setState(108);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__3) {
				{
				setState(104);
				match(T__3);
				setState(105);
				params();
				setState(106);
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
		enterRule(_localctx, 14, RULE_key);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(110);
			match(T__8);
			setState(111);
			match(ATT);
			setState(112);
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
		enterRule(_localctx, 16, RULE_params);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(114);
			bexpr();
			setState(119);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,13,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(115);
					match(T__6);
					setState(116);
					bexpr();
					}
					} 
				}
				setState(121);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,13,_ctx);
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
		enterRule(_localctx, 18, RULE_bexpr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(122);
			name();
			setState(123);
			_la = _input.LA(1);
			if ( !(_la==BOP || _la==ATT) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			setState(124);
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
		enterRule(_localctx, 20, RULE_value);
		try {
			setState(128);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case FLOAT:
				enterOuterAlt(_localctx, 1);
				{
				setState(126);
				match(FLOAT);
				}
				break;
			case WORD:
			case WS:
				enterOuterAlt(_localctx, 2);
				{
				setState(127);
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
		enterRule(_localctx, 22, RULE_name);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(133);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==WS) {
				{
				{
				setState(130);
				match(WS);
				}
				}
				setState(135);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(136);
			match(WORD);
			setState(140);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==WS) {
				{
				{
				setState(137);
				match(WS);
				}
				}
				setState(142);
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
		"\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\3\24\u0092\4\2\t\2"+
		"\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13"+
		"\t\13\4\f\t\f\4\r\t\r\3\2\3\2\3\2\3\2\3\2\3\2\7\2!\n\2\f\2\16\2$\13\2"+
		"\3\2\7\2\'\n\2\f\2\16\2*\13\2\3\2\7\2-\n\2\f\2\16\2\60\13\2\3\2\5\2\63"+
		"\n\2\3\2\5\2\66\n\2\6\28\n\2\r\2\16\29\3\2\3\2\3\3\3\3\5\3@\n\3\3\4\3"+
		"\4\3\4\6\4E\n\4\r\4\16\4F\3\4\3\4\3\5\3\5\3\5\5\5N\n\5\3\5\3\5\3\6\3\6"+
		"\3\6\3\6\3\6\3\6\5\6X\n\6\3\7\3\7\3\7\3\7\3\7\3\7\7\7`\n\7\f\7\16\7c\13"+
		"\7\3\7\3\7\5\7g\n\7\3\b\3\b\3\b\3\b\3\b\3\b\5\bo\n\b\3\t\3\t\3\t\3\t\3"+
		"\n\3\n\3\n\7\nx\n\n\f\n\16\n{\13\n\3\13\3\13\3\13\3\13\3\f\3\f\5\f\u0083"+
		"\n\f\3\r\7\r\u0086\n\r\f\r\16\r\u0089\13\r\3\r\3\r\7\r\u008d\n\r\f\r\16"+
		"\r\u0090\13\r\3\r\2\2\16\2\4\6\b\n\f\16\20\22\24\26\30\2\3\3\2\r\16\2"+
		"\u0097\2\67\3\2\2\2\4?\3\2\2\2\6A\3\2\2\2\bM\3\2\2\2\nQ\3\2\2\2\fY\3\2"+
		"\2\2\16h\3\2\2\2\20p\3\2\2\2\22t\3\2\2\2\24|\3\2\2\2\26\u0082\3\2\2\2"+
		"\30\u0087\3\2\2\2\32\33\7\3\2\2\33\34\5\30\r\2\34\35\7\4\2\2\35\36\7\23"+
		"\2\2\36\"\5\4\3\2\37!\5\n\6\2 \37\3\2\2\2!$\3\2\2\2\" \3\2\2\2\"#\3\2"+
		"\2\2#(\3\2\2\2$\"\3\2\2\2%\'\5\f\7\2&%\3\2\2\2\'*\3\2\2\2(&\3\2\2\2()"+
		"\3\2\2\2).\3\2\2\2*(\3\2\2\2+-\5\16\b\2,+\3\2\2\2-\60\3\2\2\2.,\3\2\2"+
		"\2./\3\2\2\2/\62\3\2\2\2\60.\3\2\2\2\61\63\7\22\2\2\62\61\3\2\2\2\62\63"+
		"\3\2\2\2\63\65\3\2\2\2\64\66\7\24\2\2\65\64\3\2\2\2\65\66\3\2\2\2\668"+
		"\3\2\2\2\67\32\3\2\2\289\3\2\2\29\67\3\2\2\29:\3\2\2\2:;\3\2\2\2;<\7\2"+
		"\2\3<\3\3\2\2\2=@\5\b\5\2>@\5\6\4\2?=\3\2\2\2?>\3\2\2\2@\5\3\2\2\2AB\7"+
		"\f\2\2BD\7\23\2\2CE\5\4\3\2DC\3\2\2\2EF\3\2\2\2FD\3\2\2\2FG\3\2\2\2GH"+
		"\3\2\2\2HI\7\24\2\2I\7\3\2\2\2JN\5\f\7\2KN\5\n\6\2LN\5\16\b\2MJ\3\2\2"+
		"\2MK\3\2\2\2ML\3\2\2\2NO\3\2\2\2OP\7\22\2\2P\t\3\2\2\2QR\7\5\2\2RW\5\30"+
		"\r\2ST\7\6\2\2TU\5\22\n\2UV\7\7\2\2VX\3\2\2\2WS\3\2\2\2WX\3\2\2\2X\13"+
		"\3\2\2\2YZ\7\b\2\2Zf\5\30\r\2[\\\7\6\2\2\\a\5\20\t\2]^\7\t\2\2^`\5\22"+
		"\n\2_]\3\2\2\2`c\3\2\2\2a_\3\2\2\2ab\3\2\2\2bd\3\2\2\2ca\3\2\2\2de\7\7"+
		"\2\2eg\3\2\2\2f[\3\2\2\2fg\3\2\2\2g\r\3\2\2\2hi\7\n\2\2in\5\30\r\2jk\7"+
		"\6\2\2kl\5\22\n\2lm\7\7\2\2mo\3\2\2\2nj\3\2\2\2no\3\2\2\2o\17\3\2\2\2"+
		"pq\7\13\2\2qr\7\16\2\2rs\5\30\r\2s\21\3\2\2\2ty\5\24\13\2uv\7\t\2\2vx"+
		"\5\24\13\2wu\3\2\2\2x{\3\2\2\2yw\3\2\2\2yz\3\2\2\2z\23\3\2\2\2{y\3\2\2"+
		"\2|}\5\30\r\2}~\t\2\2\2~\177\5\26\f\2\177\25\3\2\2\2\u0080\u0083\7\17"+
		"\2\2\u0081\u0083\5\30\r\2\u0082\u0080\3\2\2\2\u0082\u0081\3\2\2\2\u0083"+
		"\27\3\2\2\2\u0084\u0086\7\21\2\2\u0085\u0084\3\2\2\2\u0086\u0089\3\2\2"+
		"\2\u0087\u0085\3\2\2\2\u0087\u0088\3\2\2\2\u0088\u008a\3\2\2\2\u0089\u0087"+
		"\3\2\2\2\u008a\u008e\7\20\2\2\u008b\u008d\7\21\2\2\u008c\u008b\3\2\2\2"+
		"\u008d\u0090\3\2\2\2\u008e\u008c\3\2\2\2\u008e\u008f\3\2\2\2\u008f\31"+
		"\3\2\2\2\u0090\u008e\3\2\2\2\23\"(.\62\659?FMWafny\u0082\u0087\u008e";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}