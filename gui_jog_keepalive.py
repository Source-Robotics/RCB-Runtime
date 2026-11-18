"""A held jog's dead-man, the dashboard's half (2026-11-18, TODO/review_2026-11-18.md B4 + G3; user: "RTI + dashboard").

While a jog button or joystick (any element with the class `rcb-jog`) is pressed, the BROWSER sends `rcb_jog_alive`
every 0.2 s; each becomes one `jog_alive()` to the executor, which stops a jog of this connection whose keep-alives
stopped for 0.6 s (command_executor.JOG_ALIVE_S). Sent from the browser, not from a server timer, so a lost button
release (a context menu, the tab switched or hidden) or a lost connection stops the joint. Pointer events: a touch
screen holds a jog the same way.
"""
from __future__ import annotations

JS = """<script>
(function () {
  if (window.rcbJogKeepAlive) return;
  window.rcbJogKeepAlive = true;
  let timer = null;
  const stop = () => { if (timer) { clearInterval(timer); timer = null; } };
  document.addEventListener('pointerdown', (e) => {
    if (e.button !== 0 || !e.target.closest || !e.target.closest('.rcb-jog')) return;
    stop();
    emitEvent('rcb_jog_alive');
    timer = setInterval(() => emitEvent('rcb_jog_alive'), 200);
  }, true);
  for (const name of ['pointerup', 'pointercancel', 'contextmenu']) document.addEventListener(name, stop, true);
  window.addEventListener('blur', stop);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
})();
</script>"""

EVENT = "rcb_jog_alive"


def install(exec_fn):
    """On this page: the script, and each keep-alive passed on as jog_alive. exec_fn(fn, label): the page's _exec."""
    from nicegui import ui
    ui.add_body_html(JS)
    ui.on(EVENT, lambda e: exec_fn(lambda r: r.jog_alive(), "jog_alive"))
