#!/usr/bin/env bash
# HTTP smoke tests for control_demo_web. Run while launch is running.
set -u

BASE="${BASE_URL:-http://127.0.0.1:8000}"
PASS=0
FAIL=0

check() {
    local name="$1"
    local expected="$2"
    local actual="$3"
    if [[ "$actual" == "$expected" ]]; then
        echo "PASS  $name -> $actual"
        PASS=$((PASS + 1))
    else
        echo "FAIL  $name -> $actual (expected $expected)"
        FAIL=$((FAIL + 1))
    fi
}

code() {
    curl -sS -o /dev/null -w '%{http_code}' "$@"
}

check "GET /" 200 "$(code "$BASE/")"
check "GET /style.css" 200 "$(code "$BASE/style.css")"
check "GET /app.js" 200 "$(code "$BASE/app.js")"
check "GET /joystick.js" 200 "$(code "$BASE/joystick.js")"
check "GET /api/status" 200 "$(code "$BASE/api/status")"
check "GET /api/health" 200 "$(code "$BASE/api/health")"
check "GET missing" 404 "$(code "$BASE/no-such-file")"
check "GET traversal" 404 "$(code --path-as-is "$BASE/../../etc/passwd")"

check "POST joy valid" 200 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"forward":0.5,"yaw":0.1,"strafe":-0.2,"head":0.0,"seq":1}' \
    "$BASE/api/joy")"
check "POST joy clamp/NaN" 400 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"forward":NaN,"yaw":0,"strafe":0,"head":0}' \
    "$BASE/api/joy")"
check "POST joy bad json" 400 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{bad json' "$BASE/api/joy")"
check "POST joy wrong type" 415 "$(code -X POST \
    -H 'Content-Type: text/plain' -d 'forward=0.5' "$BASE/api/joy")"
check "POST command valid" 200 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"command":"stand_up"}' "$BASE/api/command")"
check "POST command invalid" 400 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"command":"jump"}' "$BASE/api/command")"
check "POST estop" 200 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{}' "$BASE/api/estop")"
check "POST estop clear" 202 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{}' "$BASE/api/estop/clear")"
check "POST unknown endpoint" 404 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{}' "$BASE/api/nope")"

BIG_BODY="$(python3 -c "print('{\"forward\":' + '1' * 5000 + '}')")"
check "POST joy oversized" 413 "$(code -X POST \
    -H 'Content-Type: application/json' --data-binary "$BIG_BODY" "$BASE/api/joy")"

echo
echo "Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
