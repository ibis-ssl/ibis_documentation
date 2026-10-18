# ibis Documentation

RoboCup SSL チーム ibis の公開ドキュメント（https://ibis-ssl.github.io/ibis_documentation/）。
[Zensical](https://zensical.org/) でビルドし、`main` への push で GitHub Pages に公開されます。
仕様の正本は [ibis-ssl/specs](https://github.com/ibis-ssl/specs) で、他リポジトリの文書とあわせてビルド時に取得します（`scripts/fetch_docs.sh`）。
元のリポジトリの更新に追従するため毎日再ビルドします。急いで反映したいときは Actions の Documentation ワークフローを手動実行してください。

## ローカルでのプレビュー

```bash
scripts/fetch_docs.sh
uvx --with mdx_truly_sane_lists --with markdown-callouts zensical serve
```

## 構成

- `zensical.toml` — サイト設定
- `docs/index.md` — トップページ
- `docs/stylesheets/extra.css` — テーマのカスタマイズ
- `docs/specs/` — [specs](https://github.com/ibis-ssl/specs) をビルド時に取得する領域（git 管理外）
- `docs/others/<repo>/` — 他リポジトリの `doc/`・`docs/` をビルド時に取得する領域（git 管理外）。索引は `docs/others/index.md`
- `scripts/fetch_docs.sh` — 取得するリポジトリの一覧と取得処理。リポジトリを追加するときはここと `docs/others/index.md` に追記する

取得した領域は直接編集せず、元のリポジトリを更新してください。
