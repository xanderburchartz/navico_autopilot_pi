#!/usr/bin/env node
// Simulated gateway for the NMEA 0183 ways: a NAC-3 at address 4 whose
// messages go out as $PCDIN or $MXPGN sentences over TCP, and that takes
// 130850 commands wrapped the same way. For testing the plugin without a boat.
//
//   node test/sim/wrap0183_sim.js pcdin [port] [alarm]
//   node test/sim/wrap0183_sim.js mxpgn [port] [alarm]
//
// Sends every second: 65305 (pilot mode) and 127237 (heading to steer) from
// address 4, in Wind mode also 65341 (wind angle to hold, 40 degrees to
// starboard). With "alarm": an active "Rudder limit" alarm (130850 + 130856)
// every 8.8 s, as a NAC-3 repeats it, until it is acknowledged (state 0x38);
// then it is cleared (0x3A). Also a rudder angle (127245, 1 degree to
// starboard) and a log with 5.2 kn through the water (128259, address 35), and
// $IIHDM with the heading for OpenCPN. Logs every command it receives and
// changes the mode as a NAC-3 does. Nothing here talks to a real bus.

'use strict'
const net = require('net')

const kind = process.argv[2] === 'mxpgn' ? 'mxpgn' : 'pcdin'
const port = Number(process.argv[3] || 10111)
const withAlarm = process.argv[4] === 'alarm'
const PILOT = 4
const ALARM_ID = 0x3f // Rudder limit
const ALARM_TEXT = 'Rudder limit'
let alarmState = withAlarm ? 0x39 : 0 // 0x39 active, 0x3A cleared, 0: none

const hex2 = (n) => n.toString(16).toUpperCase().padStart(2, '0')
const checksum = (s) => {
  let c = 0
  for (const ch of s) c ^= ch.charCodeAt(0)
  return hex2(c)
}
const sentence = (body) => `$${body}*${checksum(body)}\r\n`
const log = (...a) => console.log(new Date().toISOString(), ...a)

// Mode bits of 65305 as a NAC-3 sends them (test/test_simnet.cpp).
const MODES = { standby: 0x0008, heading: 0x0010, wind: 0x0400, nav: 0x0040, nodrift: 0x0100 }
const EVENTS = { 0x06: 'standby', 0x09: 'heading', 0x0f: 'wind', 0x0a: 'nav', 0x0c: 'nodrift' }
let mode = 'standby'
let target = 90 // degrees, magnetic

const FAST = new Set([130850, 130851, 130856, 130845, 130846, 126996, 127237, 129284, 129285])

function encode (pgn, prio, src, data, seq) {
  if (kind === 'pcdin') {
    const p = pgn.toString(16).toUpperCase().padStart(6, '0')
    return [sentence(`PCDIN,${p},00000000,${hex2(src)},${data.map(hex2).join('')}`)]
  }
  const frames = []
  if (!FAST.has(pgn) && data.length <= 8) {
    frames.push(data)
  } else {
    let i = 0
    let f = [(seq & 7) << 5, data.length]
    while (i < data.length && f.length < 8) f.push(data[i++])
    while (f.length < 8) f.push(0xff)
    frames.push(f)
    for (let n = 1; i < data.length; n++) {
      f = [((seq & 7) << 5) | n]
      while (i < data.length && f.length < 8) f.push(data[i++])
      while (f.length < 8) f.push(0xff)
      frames.push(f)
    }
  }
  return frames.map((f) => {
    const attr = ((prio & 7) << 12) | (f.length << 8) | src // send bit 0: from the bus
    const p = pgn.toString(16).toUpperCase().padStart(6, '0')
    const d = f.slice().reverse().map(hex2).join('')
    return sentence(`MXPGN,${p},${attr.toString(16).toUpperCase().padStart(4, '0')},${d}`)
  })
}

const partial = new Map()
function decode (line) {
  const m = /^\$(PCDIN|MXPGN),([^*]*)\*([0-9A-Fa-f]{2})/.exec(line.trim())
  if (!m) return null
  if (checksum(`${m[1]},${m[2]}`) !== m[3].toUpperCase()) {
    log('checksum wrong:', line.trim())
    return null
  }
  const f = m[2].split(',')
  let pgn = parseInt(f[0], 16)
  // PDU1 (PF < 240): the low byte of the PGN field is the destination.
  if (((pgn >> 8) & 0xff) < 240) pgn &= ~0xff
  if (m[1] === 'PCDIN') {
    const data = f[3].match(/../g).map((b) => parseInt(b, 16))
    return { pgn, data }
  }
  const attr = parseInt(f[1], 16)
  const bytes = f[2].match(/../g).map((b) => parseInt(b, 16)).reverse()
  if (!(attr & 0x8000)) log('MXPGN without send bit:', line.trim())
  if (!FAST.has(pgn) && bytes.length <= 8) return { pgn, data: bytes }
  const seq = bytes[0] >> 5
  const n = bytes[0] & 0x1f
  if (n === 0) {
    partial.set(pgn, { seq, len: bytes[1], data: bytes.slice(2), next: 1 })
  } else {
    const p = partial.get(pgn)
    if (!p || p.seq !== seq || p.next !== n) { partial.delete(pgn); return null }
    p.data.push(...bytes.slice(1))
    p.next++
  }
  const p = partial.get(pgn)
  if (p && p.data.length >= p.len) {
    partial.delete(pgn)
    return { pgn, data: p.data.slice(0, p.len) }
  }
  return null
}

let seq = 0
function status () {
  const bits = MODES[mode]
  const out = encode(65305, 7, PILOT, [0x41, 0x9f, 0x64, 0x0a, bits & 0xff, bits >> 8, 0x00, 0x00], 0)
  // 127237: heading to steer in 0.0001 rad at bytes 5-6, magnetic (bits 6-7 of byte 1 = 1).
  const raw = mode === 'standby' ? 0xffff : Math.round(target * Math.PI / 180 * 10000)
  const hc = [0xff, 0x7f, 0xff, 0x7f, 0xff, raw & 0xff, raw >> 8, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
  out.push(...encode(127237, 2, PILOT, hc, seq++ & 7))
  // 127245: rudder position in 0.0001 rad at bytes 4-5, 1 degree to starboard.
  const rud = Math.round(Math.PI / 180 * 10000)
  out.push(...encode(127245, 2, PILOT, [0xff, 0xff, 0xff, 0x7f, rud & 0xff, rud >> 8, 0xff, 0xff], 0))
  // 128259: speed through the water in 0.01 m/s at bytes 1-2, 5.2 kn, from a log at 35.
  const stw = Math.round(5.2 * 1852 / 3600 * 100)
  out.push(...encode(128259, 2, 35, [0x00, stw & 0xff, stw >> 8, 0xff, 0xff, 0x00, 0xff, 0xff], 0))
  out.push(sentence(`IIHDM,${(mode === 'standby' ? 92 : target).toFixed(1)},M`))
  if (mode === 'wind') {
    // 65341, wind mode (3), 40 degrees = 6981 in 0.0001 rad.
    out.push(...encode(65341, 6, PILOT, [0x41, 0x9f, 0xff, 0xff, 0x03, 0xff, 6981 & 0xff, 6981 >> 8], 0))
  }
  return out
}

function alarm () {
  if (!alarmState) return []
  const out = encode(130850, 2, PILOT,
    [0x41, 0x9f, 0xff, 0xff, 0x01, 0xff, alarmState, 0x00, ALARM_ID, 0x00, 0x03, 0x00], seq++ & 7)
  const text = [...ALARM_TEXT].map((c) => c.charCodeAt(0))
  out.push(...encode(130856, 6, PILOT, [0x41, 0x9f, ALARM_ID, 0x00, text.length + 2, 0x01, ...text], seq++ & 7))
  if (alarmState === 0x3a) alarmState = 0 // cleared once, then quiet
  return out
}

function command (data) {
  // 130850: 41 9F <address> FF FF <type> <event> ...
  if (data[0] !== 0x41 || data[1] !== 0x9f) return
  const hex = data.map(hex2).join(' ')
  // Acknowledgement of an alarm, as a Triton2 sends it: the alarm message with state 0x38.
  if (data[2] === 0xff && data[4] === 0x01 && data[6] === 0x38) {
    const id = data[8] | (data[9] << 8)
    log(`alarm 0x${hex2(id)} acknowledged: ${hex}`)
    if (id === ALARM_ID && alarmState === 0x39) alarmState = 0x3a
    return
  }
  if (data[2] !== PILOT) { log(`130850 for address ${data[2]}, not the pilot: ${hex}`); return }
  if (data[5] === 0x02 && data[6] === 0x0d) {
    log(`NFU ${{ 4: 'port', 5: 'starboard', 255: 'stop' }[data[8]] || data[8]}: ${hex}`)
    return
  }
  if (data[5] === 0x0a && data[6] === 0x1a) {
    const deg = Math.round((data[9] | (data[10] << 8)) / 10000 * 180 / Math.PI)
    target = (target + (data[8] === 0x02 ? -deg : deg) + 360) % 360
    log(`course ${data[8] === 0x02 ? '-' : '+'}${deg} -> ${target}: ${hex}`)
    return
  }
  const name = EVENTS[data[6]]
  if (name) {
    log(`command 0x${hex2(data[6])} -> ${name}: ${hex}`)
    mode = name
    return
  }
  log(`130850 other: ${hex}`)
}

net.createServer((sock) => {
  log(`client ${sock.remoteAddress}:${sock.remotePort} (${kind})`)
  const timer = setInterval(() => {
    status().forEach((s) => sock.write(s))
    if (alarmState === 0x3a) alarm().forEach((s) => sock.write(s)) // cleared about 1 s after the ack
  }, 1000)
  const alarmTimer = setInterval(() => alarm().forEach((s) => sock.write(s)), 8800)
  alarm().forEach((s) => sock.write(s))
  let buf = ''
  sock.on('data', (d) => {
    buf += d.toString('latin1')
    let i
    while ((i = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, i)
      buf = buf.slice(i + 1)
      if (!line.trim()) continue
      const msg = decode(line)
      if (!msg) { if (!/^\$(PCDIN|MXPGN)/.test(line.trim())) log('other sentence:', line.trim()); continue }
      if (msg.pgn === 130850) command(msg.data)
      else log(`received ${msg.pgn}: ${msg.data.map(hex2).join(' ')}`)
    }
  })
  sock.on('close', () => { clearInterval(timer); clearInterval(alarmTimer); log('client closed') })
  sock.on('error', () => { clearInterval(timer); clearInterval(alarmTimer) })
}).listen(port, '127.0.0.1', () => log(`simulated ${kind} gateway on 127.0.0.1:${port}`))
