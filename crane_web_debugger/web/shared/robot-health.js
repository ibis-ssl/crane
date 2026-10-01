// ロボットの電池電圧とモータ温度の警告線。viewer・Robot Manager・Packet Forge で同じ値を使う。
// バッテリやモータの仕様が変わったら、ここだけを直す。

const VOLTAGE_CRIT_V = 21.0;
const VOLTAGE_WARN_V = 22.5;
const TEMP_CRIT_C = 75;
const TEMP_WARN_C = 60;

// severity: 'ok' | 'warn' | 'crit'。値が無いときは 'ok'
export function voltageSeverity(v) {
    if (v == null) return 'ok';
    if (v <= VOLTAGE_CRIT_V) return 'crit';
    if (v <= VOLTAGE_WARN_V) return 'warn';
    return 'ok';
}

export function temperatureSeverity(t) {
    if (t == null) return 'ok';
    if (t >= TEMP_CRIT_C) return 'crit';
    if (t >= TEMP_WARN_C) return 'warn';
    return 'ok';
}
