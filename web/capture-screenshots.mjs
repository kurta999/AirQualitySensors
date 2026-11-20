/* Read-only screenshots of the real device for README documentation. */
import { chromium } from 'playwright-core';
import { mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';

const base = process.env.AIR_DEVICE_URL ?? 'http://air.local';
const directory = new URL('../docs/screenshots/', import.meta.url);
await mkdir(directory, { recursive: true });
const browser = await chromium.launch({ channel: 'chrome', headless: true });
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1050 }, deviceScaleFactor: 1 });
  await page.goto(base, { waitUntil: 'domcontentloaded' });
  await page.getByText('Connected via USB', { exact: true }).waitFor({ timeout: 25000 });
  await page.locator('.sensor-state').filter({ hasText: 'Online' }).nth(3).waitFor({ timeout: 20000 });
  await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  await page.screenshot({ path: fileURLToPath(new URL('dashboard-overview.png', directory)) });

  await page.getByRole('link', { name: 'Measurements', exact: true }).click();
  // The app scrolls smoothly; a screenshot should capture the settled viewport.
  await page.waitForTimeout(500);
  await page.evaluate(() => {
    const heading = document.querySelector('.trends-heading');
    window.scrollTo({ top: heading.getBoundingClientRect().top + scrollY - 30, behavior: 'instant' });
  });
  await page.evaluate(() => new Promise(resolve => requestAnimationFrame(resolve)));
  await page.screenshot({ path: fileURLToPath(new URL('dashboard-measurements.png', directory)) });

  await page.getByRole('link', { name: 'Configuration', exact: true }).click();
  await page.locator('#settings-page').waitFor({ state: 'visible' });
  await page.evaluate(() => window.scrollTo({ top: 0, behavior: 'instant' }));
  await page.screenshot({ path: fileURLToPath(new URL('dashboard-configuration.png', directory)) });

  await page.setViewportSize({ width: 390, height: 844 });
  await page.getByRole('link', { name: 'Overview', exact: true }).click();
  await page.waitForTimeout(500);
  await page.screenshot({ path: fileURLToPath(new URL('dashboard-mobile.png', directory)) });
  console.log(`Saved four live-device screenshots in ${fileURLToPath(directory)}`);
} finally {
  await browser.close();
}
