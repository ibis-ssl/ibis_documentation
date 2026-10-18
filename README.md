# ibis Documentation

RoboCup SSL チーム ibis の公開ドキュメント（https://ibis-ssl.github.io/ibis_documentation/）。
[Zensical](https://zensical.org/) でビルドし、`main` への push で GitHub Pages に公開されます。
仕様の正本は [ibis-ssl/specs](https://github.com/ibis-ssl/specs) で、ビルド時に取得して `/specs/` に配置します（specs の更新に追従するため毎日再ビルドします）。

## ローカルでのプレビュー

```bash
git clone --depth 1 https://github.com/ibis-ssl/specs docs/specs
uvx --with mdx_truly_sane_lists --with markdown-callouts zensical serve
```

## 構成

- `zensical.toml` — サイト設定
- `docs/index.md` — トップページ
- `docs/stylesheets/extra.css` — テーマのカスタマイズ
- `docs/specs/` — [specs](https://github.com/ibis-ssl/specs) をビルド時に取得する領域（git 管理外）。仕様の修正は specs 側で行ってください
- `docs/others/<repo>/` — 他リポジトリのドキュメントを同期する領域。ここを直接編集せず、元のリポジトリを更新してください
  - `docs/others/crane/` — [crane](https://github.com/ibis-ssl/crane) の `docs/` から同期
