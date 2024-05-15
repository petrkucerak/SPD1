import "./globals.css";
import { meta } from "../components/meta";
import Header from "@/components/header";
import Footer from "@/components/footer";

export const metadata = {
  title: "Otužilcův deníček",
  ...meta,
};

export default function RootLayout({ children }) {
  return (
    <html lang="cs">
      <body className="w-full min-h-[100vh] flex flex-col items-center justify-between">
        <Header />
        <main>{children}</main>
        <Footer />
      </body>
    </html>
  );
}
