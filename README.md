# ibis Documentation

RoboCup SSL チーム ibis の公開ドキュメント（https://ibis-ssl.github.io/ibis_documentation/）。
[Zensical](https://zensical.org/) でビルドし、`main` への push で GitHub Pages に公開されます。

## ローカルでのプレビュー

```bash
uvx --with mdx_truly_sane_lists zensical serve
```

## 構成

- `zensical.toml` — サイト設定
- `docs/index.md` — トップページ
- `docs/stylesheets/extra.css` — テーマのカスタマイズ
- `docs/ai/` — [crane](https://github.com/ibis-ssl/crane) の `docs/` から同期される領域。ここを直接編集せず、crane 側を更新してください
