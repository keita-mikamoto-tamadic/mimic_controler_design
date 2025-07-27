# Pinocchio動力学練習プロジェクト

## ⚠️ 重要：セッション開始時の必須作業

**このプロジェクトで作業を開始する前に、必ず以下のコマンドを実行して仮想環境をアクティベートしてください：**

```bash
. bin/activate
```

または

```bash
source bin/activate
```

仮想環境がアクティベートされていないと、Pinocchioライブラリやその他の依存関係が正しく読み込まれません。

## コードの保管場所とリリースフロー

**ユーザーコードは必ず `src/` ディレクトリ内に作成・保管してください。**

他のディレクトリへのコード作成は許可されていません。

### 開発フロー
1. **プラン作成**: 新機能開発前に実装計画を立案
2. **テストフォルダ作成**: `src/test/機能名/` フォルダを作成
3. **プラン文書化**: `src/test/機能名/機能名.md` に詳細なプランを記述
4. **テスト実装**: プランに基づいて該当フォルダ内で実装・テスト
5. **レビュー**: ユーザーがテスト版を確認してOKを出す
6. **リリース**: OK後に `src/` ディレクトリにリリース版を作成

### 開発ルール
- **プラン文書**: 各機能フォルダに必ず `フォルダ名.md` を作成し、詳細プランを記述
- **既存関数の利用1**: 動力学ライブラリpinocchioのAPIをフル活用すること。
- **既存関数の利用2**: `src/` フォルダ内の実装済み関数は使用可能（編集は禁止）
- **段階的開発**: テスト → レビュー → リリースの順序を厳守

**重要**: プラン文書化とテスト版での動作確認・ユーザー承認を得てからのみ本番リリースを行う

## 作業履歴の管理

### working_historyフォルダ
セッション終了時に `working_history/` フォルダに作業履歴を記録します。

**ファイル形式**: `YYYYMMDD_HHMMSS.md`

**記載内容**:
- 本セッションでの作業内容
- 実装したファイルとその概要
- 技術的な達成事項
- 次回セッションで参照すべき項目
- 次回の作業予定
- 未解決の課題

**目的**:
- セッション間の継続性確保
- 作業進捗の可視化
- 技術的決定事項の記録
- 効率的な引き継ぎ

## プロジェクト概要



## Pinocchio公式ドキュメント

### メインAPIドキュメント
- **C++/Python API リファレンス**: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/
- **最新リリース版**: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/

### Python固有ドキュメント
- **Python API**: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_python.html
- **Python例とチュートリアル**: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_d-practical-exercises_intro.html

### GitHub
- **ソースコードとExamples**: https://github.com/stack-of-tasks/pinocchio
- **Examples ディレクトリ**: https://github.com/stack-of-tasks/pinocchio/tree/master/examples

### 主要API関数
- `pin.buildModelFromUrdf()` - URDFからモデル構築
- `pin.forwardKinematics()` - 順運動学
- `pin.computeJointJacobians()` - ヤコビアン計算
- `pin.rnea()` - 逆動力学（RNEA algorithm）
- `pin.aba()` - 順動力学（ABA algorithm）
- `pin.buildReducedModel()` - 削減モデル作成
- `pin.neutral()` - 中立構成取得
- `pin.randomConfiguration()` - ランダム構成生成