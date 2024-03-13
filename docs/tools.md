# ツールなど

## ament_clang_format

ROS 2インストールしたら自動的に使えるようになるフォーマッタ。
各階層に配置している`.clang-format`もほぼ同じ設定になっているので適宜開発時に使うと良い。

```bash
ament_clang_format --reformat <フォーマットしたいファイルかフォルダ>
```

## pre-commit

コミット前に自動でフォーマットをかけるツール。

```bash
pip install pre-commit
```

```bash
cd <craneのルート>
pre-commit install
pre-commit run -a
```

設定ファイルはこれ  
<https://github.com/ibis-ssl/crane/blob/develop/.pre-commit-config.yaml>
