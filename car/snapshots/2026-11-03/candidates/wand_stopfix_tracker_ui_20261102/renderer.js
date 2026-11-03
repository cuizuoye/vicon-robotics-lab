'use strict';
// Display-only, top-down Vicon/map XY view. Never rewrites input poses/goals.
// Positive X is right, positive Y is up; all grid lines are anchored at (0,0).
var TrackerWorldView = (() => {
  const finitePose = p => p && Number.isFinite(p.x) && Number.isFinite(p.y);
  function niceStep(value) {
    const power = 10 ** Math.floor(Math.log10(Math.max(value, .01)));
    return [1, 2, 5, 10].find(n => n * power >= value) * power;
  }
  function layout(width, height, car, goal, trace, camera) {
    const usableX = Math.max(40, width / 2 - 48), usableY = Math.max(40, height / 2 - 40);
    const aspect = usableX / usableY;
    let halfY = Math.max(3, camera.halfY || 3);
    const points = (trace || []).filter(p => Array.isArray(p) && p.length >= 2 && p.slice(0, 2).every(Number.isFinite)).slice(-100);
    for (const p of [car, goal]) if (finitePose(p)) points.push([p.x, p.y]);
    for (const p of points) halfY = Math.max(halfY, Math.abs(p[1]) / .84, Math.abs(p[0]) / (aspect * .84));
    // Zoom out only when necessary. No tracking-centred panning or auto zoom-in.
    camera.halfY = Math.ceil(halfY * 2) / 2;
    const scale = usableY / camera.halfY, major = Math.max(1, niceStep(38 / scale));
    return {width, height, scale, major, minor: major / 5,
      origin: [width / 2, height / 2],
      project: (x, y) => [width / 2 + x * scale, height / 2 - y * scale]};
  }
  function draw(ctx, width, height, car, goal, trace, camera) {
    const v = layout(width, height, car, goal, trace, camera), s = v.scale;
    const [ox, oy] = v.origin;
    ctx.fillStyle = '#242b32'; ctx.fillRect(0, 0, width, height);
    const xMin = -ox / s, xMax = (width - ox) / s, yMin = -(height - oy) / s, yMax = oy / s;
    function grid(step, color, lineWidth) {
      ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath();
      let n = 0;
      for (let x = Math.ceil(xMin / step) * step; x <= xMax && n++ < 180; x += step) {
        const p = v.project(x, 0)[0]; ctx.moveTo(p, 0); ctx.lineTo(p, height);
      }
      n = 0;
      for (let y = Math.ceil(yMin / step) * step; y <= yMax && n++ < 180; y += step) {
        const p = v.project(0, y)[1]; ctx.moveTo(0, p); ctx.lineTo(width, p);
      }
      ctx.stroke();
    }
    grid(v.minor, '#303a44', .7); grid(v.major, '#586573', 1);
    function axis(color, a, b, left, right) {
      ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.beginPath();
      ctx.moveTo(...a); ctx.lineTo(...b); ctx.moveTo(...left); ctx.lineTo(...b); ctx.lineTo(...right); ctx.stroke();
    }
    axis('#d98585', [0, oy], [width - 13, oy], [width - 23, oy - 5], [width - 23, oy + 5]);
    axis('#81c7a4', [ox, height], [ox, 13], [ox - 5, 23], [ox + 5, 23]);
    ctx.font = '13px "Microsoft YaHei",sans-serif'; ctx.fillStyle = '#b7c2cc'; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    let count = 0;
    const label = x => Math.abs(x) < 1e-8 ? '0' : Number(x.toPrecision(6)).toString();
    for (let x = Math.ceil(xMin / v.major) * v.major; x < xMax && count++ < 100; x += v.major) {
      const px = v.project(x, 0)[0];
      if (Math.abs(x) > 1e-7 && px > 24 && px < width - 45) ctx.fillText(label(x), px, oy + 8);
    }
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; count = 0;
    for (let y = Math.ceil(yMin / v.major) * v.major; y < yMax && count++ < 100; y += v.major) {
      const py = v.project(0, y)[1];
      if (Math.abs(y) > 1e-7 && py > 40 && py < height - 25) ctx.fillText(label(y), ox - 9, py);
    }
    ctx.font = 'bold 15px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'right';
    ctx.fillStyle = '#eea1a1'; ctx.fillText('+X', width - 12, oy - 15);
    ctx.fillStyle = '#9bdcba'; ctx.textAlign = 'left'; ctx.fillText('+Y', ox + 12, 20);
    ctx.strokeStyle = '#f0f5fa'; ctx.lineWidth = 1.8; ctx.beginPath();
    ctx.moveTo(ox - 6, oy); ctx.lineTo(ox + 6, oy); ctx.moveTo(ox, oy - 6); ctx.lineTo(ox, oy + 6); ctx.stroke();
    ctx.fillStyle = '#e6edf4'; ctx.font = '13px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'left'; ctx.fillText('0, 0', ox + 9, oy - 13);
    ctx.fillStyle = '#b9c8d6'; ctx.font = 'bold 14px "Microsoft YaHei",sans-serif'; ctx.fillText('VICON WORLD · XY', 15, 21);
    ctx.font = '12px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'right';
    ctx.fillText(`主格 ${label(v.major)} m · 细格 ${label(v.minor)} m`, width - 14, height - 15);
    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, width, height); ctx.clip();
    ctx.strokeStyle = '#6aabd0'; ctx.lineWidth = 1.7; ctx.beginPath();
    let begun = false;
    for (const p of (trace || []).slice(-100)) {
      if (!Array.isArray(p) || p.length < 2 || !p.slice(0, 2).every(Number.isFinite)) continue;
      const xy = v.project(p[0], p[1]); if (begun) ctx.lineTo(...xy); else {ctx.moveTo(...xy); begun = true;}
    }
    ctx.stroke();
    function body(p, isCar) {
      if (!finitePose(p) || !Number.isFinite(p.yaw)) return;
      const [x, y] = v.project(p.x, p.y), size = Math.max(18, Math.min(38, .21 * s));
      const color = isCar ? '#59c2f5' : '#f5b44e';
      ctx.save(); ctx.translate(x, y); ctx.rotate(-p.yaw); ctx.strokeStyle = color; ctx.lineWidth = 2;
      if (isCar) {
        ctx.fillStyle = '#245a7777'; ctx.fillRect(-size / 2, -size / 2, size, size);
        ctx.strokeRect(-size / 2, -size / 2, size, size);
        ctx.beginPath(); ctx.moveTo(-size / 2, -size / 2); ctx.lineTo(size / 2, size / 2);
        ctx.moveTo(-size / 2, size / 2); ctx.lineTo(size / 2, -size / 2); ctx.stroke();
      } else {
        // The same mapped target as before, depicted as a T rather than a dot.
        ctx.beginPath(); ctx.moveTo(-size * .6, 0); ctx.lineTo(size * .45, 0);
        ctx.moveTo(size * .45, -size * .5); ctx.lineTo(size * .45, size * .5); ctx.stroke();
      }
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(size * .95, 0);
      ctx.lineTo(size * .68, -4); ctx.moveTo(size * .95, 0); ctx.lineTo(size * .68, 4); ctx.stroke();
      ctx.restore(); ctx.fillStyle = color; ctx.font = 'bold 14px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = isCar ? 'top' : 'bottom';
      const labelY = isCar ? y + size / 2 + 7 : y - size / 2 - 8;
      ctx.fillText(isCar ? 'car01' : 'Sign目标', Math.max(5, Math.min(width - 80, x + 13)), Math.max(42, Math.min(height - 24, labelY)));
    }
    body(goal, false); body(car, true); ctx.restore();
    return v;
  }
  return {layout, draw};
})();
if (typeof module !== 'undefined' && module.exports) module.exports = TrackerWorldView;
