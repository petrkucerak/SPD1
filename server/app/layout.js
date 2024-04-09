import "./globals.css";
import { meta } from "../components/meta";

export const metadata = {
  title: "Otužilcův deníček",
  ...meta
};

export default function RootLayout({ children }) {
  return (
    <html lang="cs">
      <body>{children}</body>
    </html>
  );
}
