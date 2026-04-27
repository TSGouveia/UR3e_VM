import { useEffect, useMemo, useState } from "react";
import { API_BASE } from "./config";

function usePrefersDark() {
  const get = () => window.matchMedia?.("(prefers-color-scheme: dark)")?.matches ?? false;
  const [dark, setDark] = useState(get);

  useEffect(() => {
    const m = window.matchMedia?.("(prefers-color-scheme: dark)");
    if (!m) return;

    const onChange = () => setDark(m.matches);
    // Safari fallback
    if (m.addEventListener) m.addEventListener("change", onChange);
    else m.addListener(onChange);

    return () => {
      if (m.removeEventListener) m.removeEventListener("change", onChange);
      else m.removeListener(onChange);
    };
  }, []);

  return dark;
}

function cellKey(cell) {
  return `${cell.m}:${cell.r}-${cell.c}`;
}
function sameCell(a, b) {
  return a?.m === b?.m && a?.r === b?.r && a?.c === b?.c;
}

function Field({ label, value, onChange, min = 0, max = 99, step = 1, prefersDark }) {
  return (
    <label style={styles.field}>
      <span
        style={{
          ...styles.fieldLabel,
          ...(prefersDark && styles.titleDark),
        }}
      >
        {label}
      </span>
      <input
        type="number"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        style={{
                ...styles.input,
                ...(prefersDark && styles.inputDark),
              }}
      />
    </label>
  );
}

function Board({
  title,
  m,
  pieces,
  acceptDrops,
  onDrop,
  ghostMarkers,
  activePieceIds,
  disableDnD,
  prefersDark
}) {
  const rows = [1, 2, 3, 4];
  const cols = [1, 2];

  const piecesOnThisBoard = useMemo(() => pieces.filter((p) => p.m === m), [pieces, m]);


  const pieceByCell = useMemo(() => {
    const map = new Map();
    for (const p of piecesOnThisBoard) map.set(`${p.r}-${p.c}`, p);
    return map;
  }, [piecesOnThisBoard]);

  return (
    <div style={{
                  ...styles.card,
                  ...(prefersDark && styles.cardDark),
                }}>
      <div style={styles.cardHeader}>
        <div style={{ ...styles.cardTitle, ...(prefersDark && styles.titleDark) }}>{title}</div>
        <div style={{ ...styles.cardSubtitle, ...(prefersDark && styles.mutedDark) }}>m={m}</div>
      </div>

      <div style={{ ...styles.gridWrap, ...(prefersDark && styles.gridWrapDark) }}>
        {rows.map((r) =>
          cols.map((c) => {
            const cell = { m, r, c };
            const piece = pieceByCell.get(`${r}-${c}`);
            const ghostKind =
              Array.isArray(ghostMarkers)
                ? (ghostMarkers.find((gm) => sameCell(gm.cell, cell))?.kind ?? null)
                : null;

            const canDragThisPiece =
              !!piece && !disableDnD && (!activePieceIds?.length || activePieceIds.includes(piece.id));

            return (
              <div
                key={`${r}-${c}`}
                onDragOver={(e) => {
                  if (!acceptDrops || disableDnD) return;
                  e.preventDefault();
                }}
                onDrop={(e) => {
                  if (!acceptDrops || disableDnD) return;
                  e.preventDefault();
                  const pieceId = e.dataTransfer.getData("text/plain");
                  if (!pieceId) return;
                  onDrop?.(pieceId, cell);
                }}
                style={{
                        ...styles.cell,
                        ...(prefersDark && styles.cellDark),
                      }}
                title={`m=${m}, r=${r}, c=${c}`}
              >
                {/* ghost */}
                {ghostKind && (
                  <div
                    style={{
                      ...styles.ghost,
                      ...(ghostKind === "exec" ? styles.ghostExec : styles.ghostLocal),
                    }}
                  />
                )}

                {/* piece */}
                {piece && (
                  <div
                    draggable={canDragThisPiece}
                    onDragStart={(e) => {
                      if (!canDragThisPiece) return;
                      e.dataTransfer.setData("text/plain", piece.id);
                      e.dataTransfer.effectAllowed = "move";
                    }}
                    style={{
                      ...styles.piece,
                      cursor: canDragThisPiece ? "grab" : "not-allowed",
                      opacity: (!activePieceIds?.length || activePieceIds.includes(piece.id)) ? 1 : 0.55,
                    }}
                    title={`piece ${piece.id}`}
                  >
                    {piece.id}
                  </div>
                )}
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}


export default function App() {
  const [from, setFrom] = useState({ m: 1, r: 1, c: 1 });
  const [to, setTo] = useState({ m: 2, r: 1, c: 1 });
  const [speed, setSpeed] = useState(0.2);
  const prefersDark = usePrefersDark();

  // Requested starting poses:
  // m1 r1 c1, m1 r1 c2, m2 r1 c1, m2 r1 c2
  const [serverPieces, setServerPieces] = useState([
    { id: "A", m: 1, r: 1, c: 1 },
    { id: "B", m: 1, r: 1, c: 2 },
    { id: "C", m: 2, r: 1, c: 1 },
    { id: "D", m: 2, r: 1, c: 2 },
  ]);

  const [accepting, setAccepting] = useState(false);

  const [pending, setPending] = useState(null);
  // pending = { pieceId, from:{m,r,c}, to:{m,r,c}, sent:boolean, seenBusy:boolean }

  const [robot, setRobot] = useState({
    state: "unknown",
    current_job: null,
    current_phase: null,
    last_result: null,
    last_error: null,
    pending_move: null,
  });
  const [statusErr, setStatusErr] = useState("");

  const [sending, setSending] = useState(false);
  const [log, setLog] = useState("Idle");

  const disableDnD = useMemo(
    () => sending || accepting || (pending && pending.sent) || robot.state === "busy",
    [sending, accepting, pending, robot.state]
  );

  const canSend = useMemo(
    () => robot.state === "idle" && !sending && !!pending && !pending.sent,
    [robot.state, sending, pending]
  );

  function getPiece(board, id) {
    return board.find((p) => p.id === id) || null;
  }

  function samePos(p, cell) {
    return p && p.m === cell.m && p.r === cell.r && p.c === cell.c;
  }

  function pickBoardFromStatus(status) {
    // Show target while busy so everyone sees where it will land
    if (status.state === "busy") return status.target_board ?? status.present_board;
    return status.present_board ?? status.target_board;
  }

  function applyPending(board, pending) {
    if (!pending) return board;
    return board.map((p) =>
     (p.id === pending.pieceId ? { ...p, ...pending.to } : p)
    );
  }

const viewPieces = useMemo(() => applyPending(serverPieces, pending), [serverPieces, pending]);

  useEffect(() => {
    let cancelled = false;
    let timer = null;

    async function poll() {
      try {
        const res = await fetch(`${API_BASE}/api/status`, { method: "GET" });
        const data = await res.json().catch(() => ({}));
        if (!res.ok) throw new Error(data?.detail || `HTTP ${res.status}`);

        if (!cancelled) {
          setRobot({
            state: data.state ?? "unknown",
            current_job: data.current_job ?? null,
            current_phase: data.current_phase ?? null,
            last_result: data.last_result ?? null,
            last_error: data.last_error ?? null,
            pending_move: data.pending_move ?? null,
          });
          setStatusErr("");

          const board = pickBoardFromStatus(data);
          if (Array.isArray(board)) setServerPieces(board);

        }
      } catch (e) {
        if (!cancelled) {
          setStatusErr(e.message);
          setRobot((prev) => ({ ...prev, state: "unknown" }));
        }
      } finally {
        if (!cancelled) timer = setTimeout(poll, 500);
      }
    }

    poll();
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, []);

  useEffect(() => {
    if (!pending) return;

    const sp = getPiece(serverPieces, pending.pieceId);
    if (!sp) return;

    // If server no longer has this piece at our pending.from, then our local pending is stale.
    const serverStillAtFrom = samePos(sp, pending.from);

    if (!serverStillAtFrom) {
      // Server has moved it (either executing or already completed on another device)
      setPending(null);
      setLog(`Pending cleared (stale): piece ${pending.pieceId} changed on another device`);
    }
  }, [serverPieces, pending]);

  useEffect(() => {
    if (!pending || pending.sent) return;

    const occupiedByOther = serverPieces.some(
      (p) =>
        p.id !== pending.pieceId &&
        p.m === pending.to.m &&
        p.r === pending.to.r &&
        p.c === pending.to.c
    );

    if (occupiedByOther) {
      setPending(null);
      setLog(
        `Pending cancelled ⛔ Destination occupied: (m=${pending.to.m}, r=${pending.to.r}, c=${pending.to.c})`
      );
    }
  }, [serverPieces, pending]);

  useEffect(() => {
    if (!pending || !pending.sent) return;

    if (robot.state === "busy" && !pending.seenBusy) {
      setPending((p) => (p ? { ...p, seenBusy: true } : p));
      return;
    }

    if (robot.state === "idle" && pending.seenBusy) {
      setPending(null);
      setLog("Completed ✅ (ghost cleared)");
    }
  }, [robot.state, pending]);

  function robotHeader() {
    if (statusErr) return `Status error: ${statusErr}`;
    const job = robot.current_job ? ` (job=${robot.current_job})` : "";
    if (robot.state === "idle") return `Robot: IDLE ✅${job}`;
    if (robot.state === "busy") return `Robot: BUSY ⏳${job}`;
    if (robot.state === "error") return "Robot: ERROR ❌";
    return "Robot: UNKNOWN ⚠️";
  }

  function robotSubline() {
    if (statusErr) return null;
    if (robot.state === "busy" && robot.current_phase) return `Phase: ${robot.current_phase.replaceAll("_", " ")}`;
    if (robot.state === "error" && robot.last_error) return `Error: ${robot.last_error}`;
    if (robot.state === "idle" && robot.last_result?.message) return `Last: ${robot.last_result.message}`;
    return null;
  }

  function isOccupied(targetCell, movingPieceId) {
    const tKey = cellKey(targetCell);
    return viewPieces.some(
      (p) => p.id !== movingPieceId && cellKey({ m: p.m, r: p.r, c: p.c }) === tKey
    );
  }

  function handleDrop(pieceId, dropCell) {
    if (disableDnD) return;
    if (pending && pieceId !== pending.pieceId) return;

    if (isOccupied(dropCell, pieceId)) {
      setLog(`Blocked ⛔ Cell occupied: (m=${dropCell.m}, r=${dropCell.r}, c=${dropCell.c})`);
      return;
    }

    const piece = viewPieces.find((p) => p.id === pieceId);
    if (!piece) return;

    const nowCell = { m: piece.m, r: piece.r, c: piece.c };
    if (sameCell(nowCell, dropCell)) return;

    if (!pending) {
      const fromCell = { ...nowCell };
      const toCell = { ...dropCell };

      setFrom(fromCell);
      setTo(toCell);

      setPending({
        pieceId,
        from: fromCell,
        to: toCell,
        sent: false,
        seenBusy: false,
      });

      setLog(`Pending: ${pieceId} from (m=${fromCell.m},r=${fromCell.r},c=${fromCell.c}) -> (m=${toCell.m},r=${toCell.r},c=${toCell.c})`);
      return;
    }

    if (sameCell(dropCell, pending.from)) {
      setPending(null);
      setLog("Cancelled ✅ (returned to ghost origin, unlocked)");
      return;
    }

    const newTo = { ...dropCell };
    setTo(newTo);
    setPending((p) => (p ? { ...p, to: newTo } : p));
    setLog(`Adjusted: ${pieceId} -> (m=${newTo.m},r=${newTo.r},c=${newTo.c})`);
  }

  async function sendMove() {
    if (!pending || pending.sent) return;

    setAccepting(true);
    setSending(true);
    setLog("Sending request...");

    try {
      const res = await fetch(`${API_BASE}/api/move_piece`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ piece_id: pending.pieceId, from: pending.from, to: pending.to, speed }),
      });

      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data?.detail || data?.message || `HTTP ${res.status}`);

      setPending((p) => (p ? { ...p, sent: true } : p));
      setLog("Accepted ✅ Waiting completion...");
    } catch (e) {
      setPending(null);
      setLog(`Error ❌ ${e.message} (reverted)`);
    } finally {
      setAccepting(false);
      setSending(false);
    }
  }

  const localMove = useMemo(() => {
    if (!pending) return null;
    return { pieceId: pending.pieceId, from: pending.from, to: pending.to };
  }, [pending]);

  const execMove = useMemo(() => {
    if (robot.state !== "busy" || !robot.pending_move) return null;
    return {
      pieceId: robot.pending_move.pieceId,
      from: robot.pending_move.from,
      to: robot.pending_move.to,
    };
  }, [robot.state, robot.pending_move]);

  const activeMoves = useMemo(() => {
    const moves = [];
    if (localMove) moves.push(localMove);
    // avoid duplicate ghost if it's literally the same move/piece
    if (execMove && (!localMove || execMove.pieceId !== localMove.pieceId)) moves.push(execMove);
    return moves;
  }, [localMove, execMove]);

  const ghostMarkers = useMemo(() => {
    const out = [];
    if (localMove) out.push({ cell: localMove.from, kind: "local" });
    if (execMove && (!localMove || execMove.pieceId !== localMove.pieceId)) {
      out.push({ cell: execMove.from, kind: "exec" });
    }
    return out;
  }, [localMove, execMove]);
  const activePieceIds = activeMoves.map((m) => m.pieceId);
  

  return (
    <div style={prefersDark ? styles.pageDark : styles.page}>
      <div style={styles.shell}>
        <div style={styles.topbar}>
          <div>
            <div style={{
                      ...styles.h1,
                      ...(prefersDark && styles.textDark),
                    }}>UR3e Move Piece</div>
            <div style={{ ...styles.p, ...(prefersDark && styles.mutedDark) }}>
              Drag & drop a block to choose a move.
            </div>
          </div>

          <div style={{ ...styles.controlsCard, ...(prefersDark && styles.cardDark) }}>
            <Field prefersDark={prefersDark} label="speed" value={speed} onChange={setSpeed} min={0.05} max={1.0} step={0.05} />
            <button
              onClick={sendMove}
              disabled={!canSend}
              style={{
                ...styles.button,
                ...(prefersDark && styles.buttonDark),
                opacity: !canSend ? 0.55 : 1,
                cursor: !canSend ? "not-allowed" : "pointer",
              }}
              title={!pending ? "Drag a piece to set a move first" : !canSend ? "Robot not idle / busy / sending" : "Send move"}
            >
              {sending ? "Sending..." : pending?.sent ? "Waiting..." : "Send Move"}
            </button>
          </div>
        </div>

        <div style={{ ...styles.statusCard, ...(prefersDark && styles.cardDark) }}>
          <div style={styles.statusRow}>
            <div style={{ ...styles.statusTitle, ...(prefersDark && styles.titleDark) }}>{robotHeader()}</div>
            <div style={{
                          ...styles.badge,
                          ...(prefersDark && styles.badgeDark),
                        }}>
              {pending ? (pending.sent ? "EXECUTING" : "PENDING") : "READY"}
            </div>
          </div>

          {robotSubline() && <div style={{ ...styles.statusSub, ...(prefersDark && styles.mutedDark) }}>{robotSubline()}</div>}

          <div style={{ ...styles.statusTiny, ...(prefersDark && styles.mutedDark) }}>
            {pending ? (
              <>
                From: m={from.m}, r={from.r}, c={from.c} → To: m={to.m}, r={to.r}, c={to.c} (piece {pending.pieceId})
              </>
            ) : (
              <>Polling: {API_BASE}/api/status (every 0.5s)</>
            )}
          </div>
        </div>

        <div style={styles.boardsRow}>
          <Board
            prefersDark={prefersDark}
            title="Board 1"
            m={1}
            pieces={viewPieces}
            acceptDrops={!disableDnD}
            onDrop={handleDrop}
            ghostMarkers={ghostMarkers}
            activePieceIds={activePieceIds}
            disableDnD={disableDnD}
          />
          <Board
            prefersDark={prefersDark}
            title="Board 2"
            m={2}
            pieces={viewPieces}
            acceptDrops={!disableDnD}
            onDrop={handleDrop}
            ghostMarkers={ghostMarkers}
            activePieceIds={activePieceIds}
            disableDnD={disableDnD}
          />
        </div>

        <div style={{ ...styles.logCard, ...(prefersDark && styles.cardDark) }}>
          <div style={{ ...styles.logHeader, ...(prefersDark && styles.titleDark) }}>Log</div>
          <pre style={{
                        ...styles.pre,
                        ...(prefersDark && styles.logDark),
                      }}>{log}</pre>
        </div>
      </div>
    </div>
  );
}

const styles = {
  page: {
    minHeight: "100dvh",
    height: "100vh",
    display: "block",
    alignItems: "center",
    justifyContent: "center",
    background: "#f0f2f5",
    padding: 24,
    boxSizing: "border-box",
    overflowY: "auto",     
  },

  pageDark: {
    minHeight: "100dvh",
    height: "100vh",
    display: "block",
    alignItems: "center",
    justifyContent: "center",
    background: "#0f0f10",
    padding: 24,
    boxSizing: "border-box",
    overflowY: "auto", 
  },
  shell: {
    width: "min(980px, 100%)",
    margin: "0 auto",
    display: "grid",
    gap: 14,
  },

  topbar: {
    display: "flex",
    gap: 14,
    alignItems: "stretch",
    justifyContent: "space-between",
    flexWrap: "wrap",
  },
  h1: {
    fontSize: 24,
    fontWeight: 800,
    letterSpacing: "-0.02em",
    color: "#111827",
  },
  p: {
    marginTop: 6,
    fontSize: 13,
    color: "#4b5563",
  },

  controlsCard: {
    display: "flex",
    gap: 12,
    alignItems: "end",
    padding: 14,
    borderRadius: 14,
    background: "#fff",
    border: "1px solid rgba(17, 24, 39, 0.10)",
    boxShadow: "0 10px 22px rgba(17,24,39,0.06)",

    flexWrap: "wrap",        
    justifyContent: "flex-end",
    maxWidth: "100%",         
  },

  field: { display: "grid", gap: 6, justifyItems: "start" },
  fieldLabel: {
    fontSize: 12,
    color: "#6b7280",
    fontWeight: 700,
    letterSpacing: "0.02em",
    textTransform: "uppercase",
  },
  input: {
    width: 120,
    maxWidth: "100%",
    padding: "10px 10px",
    borderRadius: 12,
    border: "1px solid rgba(17,24,39,0.18)",
    background: "#fff",
    outline: "none",
    fontSize: 14,
  },
  button: {
    padding: "11px 16px",
    borderRadius: 12,
    border: "1px solid rgba(17,24,39,0.25)",
    background: "#111827",
    color: "white",
    fontWeight: 800,
    letterSpacing: "0.01em",

    flex: "1 1 160px",
  minWidth: 160,       
  },

  statusCard: {
    padding: 14,
    borderRadius: 14,
    background: "#fff",
    border: "1px solid rgba(17, 24, 39, 0.10)",
    boxShadow: "0 10px 22px rgba(17,24,39,0.06)",
  },
  statusRow: { display: "flex", alignItems: "center", justifyContent: "space-between", gap: 12 },
  statusTitle: { fontWeight: 900, color: "#111827" },
  statusSub: { marginTop: 6, color: "#4b5563", fontSize: 13 },
  statusTiny: { marginTop: 8, fontSize: 12, color: "#6b7280" },
  badge: {
    fontSize: 12,
    fontWeight: 900,
    letterSpacing: "0.06em",
    padding: "6px 10px",
    borderRadius: 999,
    background: "rgba(17,24,39,0.06)",
    border: "1px solid rgba(17,24,39,0.12)",
    color: "#111827",
  },

  boardsRow: {
    display: "grid",
    gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
    gap: 14,
  },

  card: {
    padding: 14,
    borderRadius: 16,
    background: "#fff",
    border: "1px solid rgba(17, 24, 39, 0.10)",
    boxShadow: "0 10px 22px rgba(17,24,39,0.06)",
  },
  cardHeader: {
    display: "flex",
    alignItems: "baseline",
    justifyContent: "space-between",
    marginBottom: 10,
  },
  cardTitle: { fontWeight: 900, color: "#111827" },
  cardSubtitle: { fontSize: 12, color: "#6b7280", fontWeight: 700 },

  gridWrap: {
    display: "grid",
    gridTemplateColumns: "repeat(2, 62px)",
    gridAutoRows: "62px",
    gap: 10,
    justifyContent: "center",
    padding: 8,
    borderRadius: 14,
    background: "rgba(17,24,39,0.03)",
    border: "1px solid rgba(17,24,39,0.06)",
  },

  cell: {
    width: 62,
    height: 62,
    borderRadius: 14,
    border: "1px solid rgba(17,24,39,0.16)",
    background: "#fff",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    position: "relative",
  },

  piece: {
    width: 40,
    height: 40,
    borderRadius: 10,
    background: "#ffd400",
    border: "1px solid #b59b00",
    boxShadow: "0 1px 2px rgba(0,0,0,0.15)",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    fontWeight: 900,
    color: "#3b2f00",
    userSelect: "none",
  },

  ghost: {
    width: 40,
    height: 40,
    borderRadius: 10,
    background: "#ffd400",
    position: "absolute",
  },

  hint: {
    marginTop: 10,
    fontSize: 12,
    color: "#6b7280",
    textAlign: "center",
  },

  logCard: {
    padding: 14,
    borderRadius: 14,
    background: "#fff",
    border: "1px solid rgba(17, 24, 39, 0.10)",
    boxShadow: "0 10px 22px rgba(17,24,39,0.06)",
  },
  logHeader: { fontWeight: 900, color: "#111827", marginBottom: 8 },
  pre: {
    margin: 0,
    padding: 12,
    borderRadius: 12,
    background: "rgba(17,24,39,0.04)",
    border: "1px solid rgba(17,24,39,0.07)",
    overflow: "auto",
    fontSize: 12,
    color: "#111827",
    whiteSpace: "pre-wrap",
  },
  ghostLocal: {
    border: "1px dashed #b59b00",
    opacity: 0.28,
  },

  ghostExec: {
    border: "1px dotted #b59b00",
    opacity: 0.28,
  },
  // ---------- DARK MODE OVERRIDES ----------
  gridWrapDark: {
    background: "rgba(255,255,255,0.06)",
    border: "1px solid rgba(255,255,255,0.10)",
  },
  buttonDark: {
    background: "#e5e7eb",
    color: "#111827",
    border: "1px solid rgba(0,0,0,0.25)",
  },
  cardDark: {
    background: "#171717",
    border: "1px solid rgba(255,255,255,0.08)",
    boxShadow: "0 10px 22px rgba(0,0,0,0.4)",
  },
  textDark: {
    color: "#e5e7eb",
  },
  mutedDark: {
    color: "#9ca3af",
  },
  cellDark: {
    background: "#f8fafc",    
    border: "1px solid rgba(0,0,0,0.18)"
  },
  inputDark: {
    background: "#111111",
    color: "#e5e7eb",
    border: "1px solid rgba(255,255,255,0.18)",
  },
  badgeDark: {
    background: "rgba(255,255,255,0.08)",
    border: "1px solid rgba(255,255,255,0.14)",
    color: "#e5e7eb",
  },
  logDark: {
    background: "rgba(255,255,255,0.06)",
    border: "1px solid rgba(255,255,255,0.12)",
    color: "#e5e7eb",
  },
  titleDark: {
    color: "#e5e7eb",
  },
};