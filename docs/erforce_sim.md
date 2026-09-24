# ER-Forceシミュレータ

Craneで使うシミュレータは [ibis-ssl/framework](https://github.com/ibis-ssl/framework) です。通常は [Docker開発環境](docker.md)から起動します。

単独ビルド・起動はframework側のREADMEと `simulator-cli --help` を参照してください。フィールド形状や物理条件の選択肢もシミュレータ側が正本です。

Craneと組み合わせる際は、送信形式・フィールド形状・Vision/Refereeの接続先を合わせ、[ネットワーク隔離](network.md)を適用してください。
